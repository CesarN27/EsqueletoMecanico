#include "imu.h"

#include <LittleFS.h>

#include "config.h"
#include "levantarse.h"
#include "log.h"
#include "motores.h"
#include "mpu6050.h"

// ---- Configuración ----
constexpr uint32_t PERIODO_MS = 10;         // 100 Hz
constexpr size_t TAMANO_MAX = 500000;       // por archivo; dos archivos caben en la flash (~1.4 MB)
constexpr int FALLOS_PARA_RECONECTAR = 20;  // lecturas seguidas fallidas (~0.2 s)
constexpr uint32_t REINTENTO_MS = 1000;
constexpr int PROFUNDIDAD_COLA = 256;       // ~2.5 s de muestras si la flash se retrasa

constexpr const char* ARCHIVO = "/imu.csv";
constexpr const char* ARCHIVO_PREV = "/imu_prev.csv";
constexpr const char* ENCABEZADO = "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,motor,enc_a,enc_b";

// Dos tareas: la de muestreo (tiempo real) lee el sensor y deja las muestras en una cola; la de
// escritura las guarda en la flash, cuyas operaciones pueden tardar decenas de ms.
struct Registro {
  uint32_t t;
  MuestraImu m;
  uint8_t motor;  // 1 = motor girando en este instante
  long encA, encB;
};

static QueueHandle_t cola = nullptr;       // hacia la escritura en flash
static QueueHandle_t colaSerie = nullptr;  // hacia el monitor serie (si se atrasa, solo pierde ahí)
static volatile bool transmitir = true;    // mostrar los datos en el monitor serie
static volatile bool pausado = false;
static volatile bool enPausaMuestreo = false;  // cada tarea confirma que ya soltó bus / archivo
static volatile bool enPausaEscritura = false;
static volatile bool conectado = false;
static volatile uint32_t descartadas = 0;  // muestras perdidas por cola llena

static size_t tamanoArchivo(const char* ruta) {
  File f = LittleFS.open(ruta, FILE_READ);
  if (!f) return 0;
  size_t n = f.size();
  f.close();
  return n;
}

static void abrirNuevo() {
  File f = LittleFS.open(ARCHIVO, FILE_WRITE);
  if (f) {
    f.println(ENCABEZADO);
    f.close();
  }
}

// El archivo actual pasa a ser el "prev" (se pierde el prev anterior) y se crea uno nuevo
static void nuevoArchivo() {
  if (LittleFS.exists(ARCHIVO_PREV)) LittleFS.remove(ARCHIVO_PREV);
  if (LittleFS.exists(ARCHIVO)) LittleFS.rename(ARCHIVO, ARCHIVO_PREV);
  abrirNuevo();
}

static void tareaMuestreo(void*) {
  uint32_t ultimoIntento = 0;
  bool primerIntento = true;
  bool avisado = false;
  int seguidos = 0;
  TickType_t t0 = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(PERIODO_MS));

    if (pausado) {
      enPausaMuestreo = true;
      t0 = xTaskGetTickCount();  // al reanudar no se recuperan los ciclos perdidos
      continue;
    }
    enPausaMuestreo = false;

    // Conexión con el sensor: se reintenta sola si falta o se pierde
    if (!conectado) {
      if (!primerIntento && millis() - ultimoIntento < REINTENTO_MS) continue;
      primerIntento = false;
      ultimoIntento = millis();
      if (!mpuIniciar()) {
        if (!avisado) {
          logPrintf("IMU: sin conexion (%s). Reintento cada 1 s; escribe t para diagnosticar\n",
                    mpuTextoError(mpuEstadisticas().ultimoError));
          avisado = true;
        }
        continue;
      }
      conectado = true;
      t0 = xTaskGetTickCount();
      avisado = false;
      seguidos = 0;
      logPrintf("IMU: conectado en 0x%02X\n", mpuDireccion());
    }

    Registro r;
    if (!mpuLeer(r.m)) {
      if (++seguidos >= FALLOS_PARA_RECONECTAR) {
        logPrintf("IMU: conexion perdida (%s). Reintentando\n",
                  mpuTextoError(mpuEstadisticas().ultimoError));
        conectado = false;
        seguidos = 0;
        primerIntento = true;
        mpuRecuperarBus();
      }
      continue;
    }
    seguidos = 0;
    r.t = millis();
    levantarseActualizar(r.t, r.m);
    r.motor = motoresGirando();
    r.encA = motoresPulsos(0);
    r.encB = motoresPulsos(1);
    if (xQueueSend(cola, &r, 0) != pdTRUE) descartadas++;
    if (transmitir) xQueueSend(colaSerie, &r, 0);
  }
}

// Cada muestra sale sin promediar (100 Hz) con el prefijo "D,". El filtro del monitor en la PC
// (monitor/filter_csv_imu.py) las guarda en un CSV y muestra una tabla de 10 Hz para leer.
static void tareaSerie(void*) {
  for (;;) {
    Registro r;
    xQueueReceive(colaSerie, &r, portMAX_DELAY);
    if (pausado || !transmitir) {
      xQueueReset(colaSerie);
      continue;
    }
    logPrintf("D,%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%u,%ld,%ld\n", (unsigned long)r.t,
              r.m.ax, r.m.ay, r.m.az, r.m.gx, r.m.gy, r.m.gz, r.m.temp, (unsigned)r.motor, r.encA,
              r.encB);
  }
}

static void tareaEscritura(void*) {
  File f;
  size_t bytes = 0;
  uint32_t ultimoFlush = 0;

  for (;;) {
    if (pausado) {
      if (f) f.close();
      xQueueReset(cola);
      enPausaEscritura = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    enPausaEscritura = false;

    Registro r;
    if (xQueueReceive(cola, &r, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!f) {
        // f.size() en modo APPEND devuelve valores falsos en este core: se mide abriendo en lectura
        bytes = tamanoArchivo(ARCHIVO);
        f = LittleFS.open(ARCHIVO, FILE_APPEND);
        if (!f) continue;
      }

      if (bytes >= TAMANO_MAX) {  // rotación: nunca se detiene el registro
        f.close();
        nuevoArchivo();
        logPrintf("IMU: archivo lleno, rotado a %s\n", ARCHIVO_PREV);
        bytes = tamanoArchivo(ARCHIVO);
        f = LittleFS.open(ARCHIVO, FILE_APPEND);
        if (!f) continue;
      }

      bytes += f.printf("%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%u,%ld,%ld\n",
                        (unsigned long)r.t, r.m.ax, r.m.ay, r.m.az, r.m.gx, r.m.gy, r.m.gz,
                        r.m.temp, (unsigned)r.motor, r.encA, r.encB);
    }

    if (f && millis() - ultimoFlush > 1000) {  // guarda cada segundo para no perder datos
      f.flush();
      ultimoFlush = millis();
    }
  }
}

void imuIniciar() {
  if (!LittleFS.begin(true)) {  // true = formatea si es la primera vez
    logPrintf("IMU: error al montar LittleFS\n");
    return;
  }

  // Cada arranque empieza una sesión nueva y la anterior pasa a "prev". Si la anterior no tiene
  // datos (solo encabezado, p. ej. un reinicio al abrir el puerto) o ya no cabe junto al nuevo, se
  // descarta SIN tocar el "prev" existente, para que un reinicio vacío nunca borre datos buenos.
  size_t previo = tamanoArchivo(ARCHIVO);
  bool conDatos = previo > strlen(ENCABEZADO) + 2 && previo <= TAMANO_MAX;
  if (previo > TAMANO_MAX)
    logPrintf("IMU: descartado %s de %u bytes (sesion anterior demasiado grande)\n", ARCHIVO,
              (unsigned)previo);
  if (conDatos) {
    nuevoArchivo();
  } else {
    if (LittleFS.exists(ARCHIVO)) LittleFS.remove(ARCHIVO);
    abrirNuevo();
  }

  logPrintf("IMU: flash %u de %u bytes usados, sesion previa en %s (%u bytes)\n",
            (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes(), ARCHIVO_PREV,
            (unsigned)tamanoArchivo(ARCHIVO_PREV));

  cola = xQueueCreate(PROFUNDIDAD_COLA, sizeof(Registro));
  colaSerie = xQueueCreate(PROFUNDIDAD_COLA, sizeof(Registro));
  xTaskCreatePinnedToCore(tareaMuestreo, "imu", 4096, nullptr, PRIO_IMU, nullptr, CORE_AUXILIAR);
  xTaskCreatePinnedToCore(tareaSerie, "imu_serie", 4096, nullptr, PRIO_IMU_SERIE, nullptr,
                          CORE_AUXILIAR);
  xTaskCreatePinnedToCore(tareaEscritura, "imu_flash", 4096, nullptr, PRIO_IMU_ESCRITURA, nullptr,
                          CORE_AUXILIAR);
}

static bool esperarPausa() {
  pausado = true;
  for (int i = 0; i < 100 && !(enPausaMuestreo && enPausaEscritura); i++)
    vTaskDelay(pdMS_TO_TICKS(5));
  return enPausaMuestreo && enPausaEscritura;
}

void imuPausar(bool pausa) {
  if (pausa) esperarPausa();
  else pausado = false;
}

bool imuEstaPausado() { return pausado; }

void imuTransmitir(bool activo) {
  if (activo && !transmitir) xQueueReset(colaSerie);
  transmitir = activo;
  if (!activo) logPrintf("IMU: transmision por serie detenida (s para reanudar). El registro en flash sigue\n");
}

bool imuTransmitiendo() { return transmitir; }

// Con el puerto reservado para que los mensajes de otras tareas no se cuelen en medio del CSV
static void volcar(const char* ruta) {
  File f = LittleFS.open(ruta, FILE_READ);
  if (!f) return;
  Serial.printf("--- INICIO %s (%u bytes) ---\n", ruta, (unsigned)f.size());
  uint8_t buf[256];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    Serial.write(buf, n);
  }
  Serial.printf("--- FIN %s ---\n", ruta);
  f.close();
}

void imuVolcarPorSerial() {
  esperarPausa();
  logBloquear();
  volcar(ARCHIVO_PREV);
  volcar(ARCHIVO);
  logLiberar();
  logPrintf("Registro en pausa. Escribe r para reanudar\n");
}

void imuBorrar() {
  esperarPausa();
  if (LittleFS.exists(ARCHIVO_PREV)) LittleFS.remove(ARCHIVO_PREV);
  abrirNuevo();
  pausado = false;
  logPrintf("IMU: archivos borrados, registrando de nuevo\n");
}

void imuEstado() {
  EstadisticasMpu& e = mpuEstadisticas();
  float tasa = e.lecturas ? 100.0f * e.fallos / e.lecturas : 0.0f;
  logPrintf("IMU estado: %s%s | lecturas=%lu fallos=%lu (%.2f%%) reconexiones=%lu descartadas=%lu\n",
            conectado ? "conectado" : "SIN CONEXION", pausado ? " (en pausa)" : "",
            (unsigned long)e.lecturas, (unsigned long)e.fallos, tasa,
            (unsigned long)e.reconexiones, (unsigned long)descartadas);
  logPrintf("IMU ultimo error: %s | %s=%u bytes, %s=%u bytes\n", mpuTextoError(e.ultimoError),
            ARCHIVO, (unsigned)tamanoArchivo(ARCHIVO), ARCHIVO_PREV,
            (unsigned)tamanoArchivo(ARCHIVO_PREV));
}
