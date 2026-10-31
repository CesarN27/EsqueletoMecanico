// Registro de un IMU MPU6050 (I2C) en CSV dentro de la flash (LittleFS).
// Guarda dos archivos: /imu.csv (sesión actual) e /imu_prev.csv (la anterior o la rotada).
#pragma once
#include <Arduino.h>

void imuIniciar();            // monta LittleFS, rota archivos y lanza la tarea de muestreo
void imuPausar(bool pausa);   // pausa / reanuda el registro (al pausar espera a que suelte el bus)
bool imuEstaPausado();
void imuTransmitir(bool activo);  // envía cada muestra por el serie a 100 Hz, con prefijo "D,"
bool imuTransmitiendo();
void imuVolcarPorSerial();    // imprime los CSV por el monitor serie (pausa el registro)
void imuBorrar();             // borra los CSV y empieza uno nuevo
void imuEstado();             // estadísticas de lecturas, fallos y reconexiones
