// Detector de "levantarse de la silla" con el IMU: enciende el motor mientras subes y lo apaga
// cuando llegas de pie. Misma logica que tools/sim_levantarse.py (ahi se prueba contra un CSV).
#pragma once
#include <Arduino.h>

#include "mpu6050.h"

// Se llama con cada muestra del IMU (100 Hz, desde la tarea de muestreo)
void levantarseActualizar(uint32_t tMs, const MuestraImu& m);

void levantarseHabilitar(bool activo);  // false = solo observa y apaga el motor
bool levantarseHabilitado();
