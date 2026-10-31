// Motores reductores con TB6612FNG y encoder, manejados con tareas FreeRTOS
#pragma once
#include <Arduino.h>

// Configura pines, PWM e interrupciones y lanza las tareas de control y de secuencia
void motoresIniciar();

// Tiempo que gira el motor en cada tramo de la secuencia (se limita al máximo de seguridad)
void motoresSetTiempoGiro(uint32_t ms);
uint32_t motoresTiempoGiro();

// Ordena a un motor (0 = A, 1 = B) ir a un ángulo absoluto en grados respecto al arranque.
// La tarea de control lo detiene al llegar o por timeout.
void motoresIrAGrados(int motor, float grados);

// Asistencia al levantarse: true = ambos motores giran en SENTIDO_LEVANTAR, false = frenan
void motoresAsistir(bool activo);

// Estado real de los motores, para registrarlo junto al IMU
bool motoresGirando();            // true si alguno tiene el PWM encendido
long motoresPulsos(int motor);    // pulsos acumulados del encoder (0 = A, 1 = B)
