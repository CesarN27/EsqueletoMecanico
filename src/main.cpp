// Firmware del exoesqueleto (ESP32) sobre FreeRTOS.
// Cada módulo crea sus propias tareas; setup() solo los inicia en orden.
//   motores  : lazo de control 1 ms (core 1) + secuencia ida/regreso
//   imu      : muestreo MPU6050 a 100 Hz a CSV en LittleFS (core 0)
//   comandos : lectura del monitor serie (core 0)
#include <Arduino.h>

#include "comandos.h"
#include "imu.h"
#include "imu_diag.h"
#include "log.h"
#include "motores.h"

void setup() {
  logIniciar();
  motoresIniciar();
  imuIniciar();
  imuDiagnostico(false);  // prueba rápida de la conexión al arrancar
  comandosIniciar();
}

// Todo el trabajo está en tareas; la tarea de Arduino ya no se necesita
void loop() { vTaskDelete(nullptr); }
