#include "imu_diag.h"

#include <Wire.h>
#include <math.h>

#include "config.h"
#include "imu.h"
#include "log.h"
#include "mpu6050.h"

static int fallas = 0;

static void resultado(bool ok, const char* prueba, const char* causa) {
  if (ok) {
    logPrintf("  [ OK  ] %s\n", prueba);
  } else {
    fallas++;
    logPrintf("  [FALLA] %s\n          -> %s\n", prueba, causa);
  }
}

// Con el pull-down interno (~45 kohm) la línea solo se queda en alto si hay un pull-up externo
// fuerte (el GY-521 trae 4.7 kohm a VCC). Eso prueba que el cable llega y que el módulo tiene VCC.
static bool hayPullup(int pin) {
  pinMode(pin, INPUT_PULLDOWN);
  delay(5);
  bool alto = digitalRead(pin);
  pinMode(pin, INPUT);
  return alto;
}

// ¿Contesta el sensor si se intercambian SDA y SCL?
static bool contestaInvertido() {
  mpuCerrarBus();
  Wire.begin(PIN_SCL, PIN_SDA);
  bool visto = mpuPresente(0x68) || mpuPresente(0x69);
  Wire.end();
  mpuAbrirBus();
  return visto;
}

static const char* nombreChip(uint8_t who) {
  switch (who) {
    case 0x68: return "MPU6050";
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x72: return "MPU6050 (clon)";
    case 0x73: return "MPU9255";
    default: return nullptr;
  }
}

struct Captura {
  int n = 0, fallos = 0, repetidas = 0, ceros = 0;
  int codigos[8] = {0};
  double sumGx = 0, sumGy = 0, sumGz = 0, sum2Gx = 0, sum2Gy = 0, sum2Gz = 0;
  double sumNorma = 0, minNorma = 1e9, maxNorma = 0, maxGiro = 0;
};

// Lee a ~100 Hz durante `ms` y acumula estadísticas
static void capturar(uint32_t ms, Captura& c) {
  MuestraImu prev = {};
  bool hayPrev = false;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    MuestraImu m;
    c.n++;
    if (!mpuLeer(m)) {
      c.fallos++;
      uint8_t e = mpuEstadisticas().ultimoError;
      c.codigos[e < 8 ? e : 0]++;
    } else {
      if (hayPrev && m.ax == prev.ax && m.ay == prev.ay && m.az == prev.az && m.gx == prev.gx &&
          m.gy == prev.gy && m.gz == prev.gz)
        c.repetidas++;
      if (m.ax == 0 && m.ay == 0 && m.az == 0 && m.gx == 0 && m.gy == 0 && m.gz == 0) c.ceros++;

      double norma = sqrt(m.ax * m.ax + m.ay * m.ay + m.az * m.az);
      double giro = sqrt(m.gx * m.gx + m.gy * m.gy + m.gz * m.gz);
      c.sumGx += m.gx; c.sum2Gx += m.gx * m.gx;
      c.sumGy += m.gy; c.sum2Gy += m.gy * m.gy;
      c.sumGz += m.gz; c.sum2Gz += m.gz * m.gz;
      c.sumNorma += norma;
      if (norma < c.minNorma) c.minNorma = norma;
      if (norma > c.maxNorma) c.maxNorma = norma;
      if (giro > c.maxGiro) c.maxGiro = giro;
      prev = m;
      hayPrev = true;
    }
    delay(10);
  }
}

static double desviacion(double suma, double suma2, int n) {
  if (n < 2) return 0;
  double media = suma / n;
  double var = suma2 / n - media * media;
  return var > 0 ? sqrt(var) : 0;
}

// Evalúa la calidad del enlace: fallos de lectura, datos congelados y datos en cero
static void evaluarEnlace(const Captura& c) {
  char causa[160];
  int buenas = c.n - c.fallos;

  int cod = 0;
  for (int i = 1; i < 8; i++)
    if (c.codigos[i] > c.codigos[cod]) cod = i;
  snprintf(causa, sizeof(causa),
           "%d de %d lecturas fallaron (%s). Cable SDA/SCL/VCC/GND flojo: muevelos para confirmar",
           c.fallos, c.n, mpuTextoError(cod));
  resultado(c.fallos == 0, "4. Lecturas sin errores (comunicacion estable)", causa);

  resultado(buenas == 0 || c.repetidas < buenas / 4, "5. Datos cambian (no congelados)",
            "Muestras identicas seguidas: el sensor dejo de actualizar (reposo/sleep o reloj parado)");
  resultado(buenas == 0 || c.ceros < buenas / 2, "6. Datos distintos de cero",
            "Todo en 0: el sensor esta dormido o el bus devuelve ceros (revisa VCC/GND)");
}

void imuDiagnostico(bool completo) {
  bool estabaPausado = imuEstaPausado();
  imuPausar(true);
  fallas = 0;
  char causa[200];

  logPrintf("\n=== DIAGNOSTICO IMU (%s) === SDA=GPIO%d SCL=GPIO%d\n", completo ? "completo" : "rapido",
            PIN_SDA, PIN_SCL);

  // 1) Eléctrico: pull-ups
  mpuCerrarBus();
  bool pSda = hayPullup(PIN_SDA);
  bool pScl = hayPullup(PIN_SCL);
  mpuAbrirBus();
  resultado(pSda, "1a. Pull-up en SDA",
            "SDA sin pull-up: el cable SDA no llega al GY-521, o el modulo no tiene VCC, o SDA va a GND");
  resultado(pScl, "1b. Pull-up en SCL",
            "SCL sin pull-up: el cable SCL no llega al GY-521, o el modulo no tiene VCC, o SCL va a GND");

  // 2) Escaneo del bus
  bool vistoSensor = mpuPresente(0x68) || mpuPresente(0x69);
  if (vistoSensor) {
    logPrintf("  [ OK  ] 2. El sensor contesta en 0x%02X\n", mpuPresente(0x68) ? 0x68 : 0x69);
  } else {
    int otra = -1;
    for (int d = 8; d < 120 && otra < 0; d++)
      if (mpuPresente(d)) otra = d;

    if (!pSda || !pScl) {
      snprintf(causa, sizeof(causa),
               "Sin pull-up en %s%s%s: ese cable no llega al modulo. Revisa tambien VCC (3V3) y GND",
               !pSda ? "SDA" : "", (!pSda && !pScl) ? " y " : "", !pScl ? "SCL" : "");
    } else if (contestaInvertido()) {
      snprintf(causa, sizeof(causa),
               "SDA y SCL estan INVERTIDOS: el sensor contesta al cambiarlos. Intercambia los cables");
    } else if (otra >= 0) {
      snprintf(causa, sizeof(causa),
               "Contesta algo en 0x%02X, no en 0x68/0x69: no es un MPU6050 o AD0 esta mal conectado",
               otra);
    } else {
      snprintf(causa, sizeof(causa),
               "Las lineas tienen pull-up pero el chip no contesta: revisa GND del modulo, y que no este danado");
    }
    resultado(false, "2. El sensor contesta en 0x68/0x69", causa);
  }

  if (!vistoSensor) {
    logPrintf("  Pruebas 3-6 omitidas (no hay sensor que probar)\n");
  } else {
    // 3) Identidad y registros
    bool iniciado = mpuIniciar();
    snprintf(causa, sizeof(causa), "No se pudo configurar: %s",
             mpuTextoError(mpuEstadisticas().ultimoError));
    resultado(iniciado, "3a. Despertar y configurar (escribe y verifica registros)", causa);

    uint8_t who = 0;
    if (mpuLeerReg(REG_WHO_AM_I, who)) {
      const char* nombre = nombreChip(who);
      if (nombre) logPrintf("  [ OK  ] 3b. WHO_AM_I = 0x%02X (%s)\n", who, nombre);
      else {
        snprintf(causa, sizeof(causa),
                 "WHO_AM_I = 0x%02X no es un MPU conocido%s", who,
                 (who == 0x00 || who == 0xFF) ? " (bus con ruido o sensor danado)" : " (clon distinto)");
        resultado(false, "3b. WHO_AM_I", causa);
      }
    } else {
      resultado(false, "3b. WHO_AM_I", "No se pudo leer el registro de identidad");
    }

    if (iniciado) {
      // 4-6) Dinámicas
      Captura c;
      if (completo) {
        logPrintf("  >> Deja el sensor QUIETO y apoyado 2 s...\n");
        capturar(2000, c);
        evaluarEnlace(c);

        int n = c.n - c.fallos;
        if (n > 0) {
          double mGx = c.sumGx / n, mGy = c.sumGy / n, mGz = c.sumGz / n;
          double norma = c.sumNorma / n;
          double ruido = fmax(desviacion(c.sumGx, c.sum2Gx, n),
                              fmax(desviacion(c.sumGy, c.sum2Gy, n), desviacion(c.sumGz, c.sum2Gz, n)));
          logPrintf("     reposo: |a|=%.3f g  gyro medio=(%.1f, %.1f, %.1f) dps  ruido=%.2f dps\n",
                    norma, mGx, mGy, mGz, ruido);
          snprintf(causa, sizeof(causa),
                   "|a|=%.2f g (esperado 1.00 g quieto): el sensor se movia o la escala esta mal", norma);
          resultado(fabs(norma - 1.0) < 0.15 && ruido < 2.0, "7. Reposo: gravedad = 1 g y giroscopio estable",
                    causa);
        }

        Captura mov;
        logPrintf("  >> AHORA AGITA / gira el sensor 3 s...\n");
        capturar(3000, mov);
        logPrintf("     movimiento: gyro max=%.0f dps  |a| de %.2f a %.2f g\n", mov.maxGiro,
                  mov.minNorma, mov.maxNorma);
        resultado(mov.maxGiro > 40 || (mov.maxNorma - mov.minNorma) > 0.3, "8. Responde al movimiento",
                  "No se detecto movimiento: no lo agitaste, o los datos no reaccionan (sensor congelado)");
      } else {
        capturar(1000, c);
        evaluarEnlace(c);
      }
    }
  }

  if (fallas == 0) logPrintf("RESULTADO: TODO OK, conexion ESP32 <-> IMU correcta\n");
  else logPrintf("RESULTADO: %d prueba(s) con FALLA, ver causas arriba\n", fallas);
  logPrintf("=== FIN DIAGNOSTICO ===\n\n");

  imuPausar(estabaPausado);
}
