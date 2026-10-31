#include "comandos.h"

#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "imu_diag.h"
#include "levantarse.h"
#include "log.h"
#include "motores.h"

static void atenderLinea() {
  char c = Serial.peek();
  if (isalpha(c)) {
    Serial.read();
    while (Serial.available()) Serial.read();
    if (c == 'v') imuVolcarPorSerial();
    else if (c == 'x') imuBorrar();
    else if (c == 'p') imuPausar(true);
    else if (c == 'r') imuPausar(false);
    else if (c == 't') imuDiagnostico(true);
    else if (c == 'e') imuEstado();
    else if (c == 's') imuTransmitir(!imuTransmitiendo());
    else if (c == 'l') levantarseHabilitar(!levantarseHabilitado());
    return;
  }

  long ms = Serial.parseInt();
  while (Serial.available()) Serial.read();  // descarta el resto de la línea
  if (ms <= 0) return;
  motoresSetTiempoGiro(ms);
  logPrintf("Nuevo tiempo de giro: %lu ms\n", (unsigned long)motoresTiempoGiro());
}

static void tareaComandos(void*) {
  for (;;) {
    if (Serial.available()) atenderLinea();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void comandosIniciar() {
  logPrintf("Escribe milisegundos (ej. 300) para cambiar el tiempo de giro\n");
  logPrintf("IMU: s = mostrar/ocultar datos, t = diagnostico completo, e = estado, l = activar/desactivar asistencia al levantarse, v = volcar CSV, x = borrar, p = pausar, r = reanudar\n");
  xTaskCreatePinnedToCore(tareaComandos, "comandos", 6144, nullptr, PRIO_COMANDOS, nullptr,
                          CORE_AUXILIAR);
}
