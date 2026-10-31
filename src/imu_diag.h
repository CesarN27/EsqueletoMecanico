// Pruebas automáticas de la conexión ESP32 <-> MPU6050.
// Estáticas: pull-ups de SDA/SCL, escaneo del bus, SDA/SCL invertidos, WHO_AM_I, registros.
// Dinámicas: tasa de fallos de lectura, datos congelados o en cero, reposo (1 g) y movimiento.
#pragma once

// completo = false: rápida (~2 s), no requiere mover el sensor (se usa al arrancar)
// completo = true : pide dejar el sensor quieto 2 s y luego agitarlo 3 s
// Pausa el registro mientras corre y lo deja como estaba.
void imuDiagnostico(bool completo);
