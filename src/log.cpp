#include "log.h"

#include <Arduino.h>
#include <stdarg.h>

static SemaphoreHandle_t mutexSerie = nullptr;

void logIniciar() {
  Serial.setTxBufferSize(1024);  // amortigua los 100 muestras/s del IMU
  Serial.begin(115200);
  mutexSerie = xSemaphoreCreateMutex();
}

void logBloquear() { xSemaphoreTake(mutexSerie, portMAX_DELAY); }

void logLiberar() { xSemaphoreGive(mutexSerie); }

void logPrintf(const char* fmt, ...) {
  char buf[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  xSemaphoreTake(mutexSerie, portMAX_DELAY);
  Serial.print(buf);
  xSemaphoreGive(mutexSerie);
}
