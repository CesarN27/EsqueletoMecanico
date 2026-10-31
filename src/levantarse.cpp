#include "levantarse.h"

#include <math.h>

#include "config.h"
#include "log.h"
#include "motores.h"

// ---- Parametros (ajustados con datos/captura_ultima3.csv) ----
// Angulo del muslo = atan2(ax, -az) (giro sobre el eje Y del IMU); subir = angulo y gy positivos.
constexpr float ALFA_GY = 0.15f;       // filtro pasa-bajos de gy (100 Hz -> ~2.8 Hz)
constexpr float ALFA_TH = 0.10f;       // filtro pasa-bajos del angulo
constexpr float GY_SUBE = 10.0f;       // dps: gy filtrado por encima = subiendo
constexpr float GY_PARA = 3.0f;        // dps: por debajo = ya no sube
constexpr float MS_CONFIRMAR = 80.0f;  // gy > GY_SUBE sostenido para encender
constexpr float MS_PARAR = 100.0f;     // gy < GY_PARA sostenido para apagar
constexpr float ARRANQUE_DEG = 2.0f;   // el angulo ya debe haber subido esto sobre el valle
constexpr float SUBIDA_MIN_DEG = 8.0f; // subida minima para contar como levantarse
constexpr float BAJA_DEG = 5.0f;       // desde el pico: ya te estas sentando -> re-armar
constexpr float CAIDA_PICO_DEG = 3.0f; // apaga si el angulo cae esto desde el pico
constexpr float FUGA_DEG_S = 0.3f;     // el valle sube lento para seguir la deriva
constexpr uint32_t MOTOR_MAX_MS = 4000;  // seguridad: nunca mas de esto encendido seguido

enum Estado { ESPERA, LEVANTANDO, ARRIBA };

static volatile bool habilitado = true;
static Estado estado = ESPERA;
static bool primera = true;
static uint32_t tAnterior = 0, tInicio = 0;
static float thf, gyf, valle, pico, conf, parar;

static void apagar(const char* motivo) {
  motoresAsistir(false);
  logPrintf("Levantarse: MOTOR OFF (%s)\n", motivo);
}

void levantarseHabilitar(bool activo) {
  habilitado = activo;
  if (!activo) {
    if (estado == LEVANTANDO) motoresAsistir(false);
    estado = ESPERA;
    primera = true;
  }
  logPrintf("Levantarse: %s\n", activo ? "activado" : "desactivado (motor apagado)");
}

bool levantarseHabilitado() { return habilitado; }

void levantarseActualizar(uint32_t tMs, const MuestraImu& m) {
  if (!habilitado) return;

  float th = atan2f(m.ax, -m.az) * 57.29578f;
  if (primera) {
    thf = valle = pico = th;
    gyf = conf = parar = 0;
    estado = ESPERA;
    primera = false;
    tAnterior = tMs;
  }
  float dt = (float)(tMs - tAnterior);
  tAnterior = tMs;
  thf += ALFA_TH * (th - thf);
  gyf += ALFA_GY * (m.gy - gyf);

  switch (estado) {
    case ESPERA:
      valle = thf < valle ? thf : valle + FUGA_DEG_S * dt / 1000.0f;
      conf = gyf > GY_SUBE ? conf + dt : 0;
      if (conf >= MS_CONFIRMAR && thf - valle >= ARRANQUE_DEG) {
        estado = LEVANTANDO;
        pico = thf;
        tInicio = tMs;
        parar = 0;
        motoresAsistir(true);
        logPrintf("Levantarse: MOTOR ON (subiendo)\n");
      }
      break;

    case LEVANTANDO: {
      if (thf > pico) pico = thf;
      parar = (gyf < GY_PARA || thf < pico - CAIDA_PICO_DEG) ? parar + dt : 0;
      bool tiempo = tMs - tInicio > MOTOR_MAX_MS;
      if (parar >= MS_PARAR || tiempo) {
        if (pico - valle >= SUBIDA_MIN_DEG || tiempo) {
          estado = ARRIBA;
          apagar(tiempo ? "limite de tiempo" : "ya estas de pie");
        } else {
          estado = ESPERA;
          valle = thf;
          conf = 0;
          apagar("falsa alarma, movimiento muy pequeno");
        }
      }
      break;
    }

    case ARRIBA:
      if (thf > pico) pico = thf;
      if (thf < pico - BAJA_DEG) {  // te sientas: listo para el siguiente levantarse
        estado = ESPERA;
        valle = thf;
        conf = 0;
      }
      break;
  }
}
