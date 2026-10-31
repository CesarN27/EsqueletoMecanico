// Pines y constantes compartidas del exoesqueleto (ESP32)
#pragma once
#include <Arduino.h>

// ---- Motores (TB6612FNG + encoder) ----
constexpr int PIN_STBY = 14;

constexpr int PWM_FREQ_HZ = 20000;
constexpr int PWM_BITS = 8;

// Ranuras del disco del encoder por vuelta del eje donde está montado el disco.
// Los discos de los motores TT suelen tener 20. Calibrar: girar a mano una vuelta y contar.
constexpr int PULSOS_POR_VUELTA = 20;

constexpr int VELOCIDAD = 150;            // PWM 0..255 durante el movimiento
constexpr uint32_t ANTIREBOTE_US = 1000;  // ignora pulsos más cercanos que esto
constexpr uint32_t TIMEOUT_MS = 3000;     // corta si no llega (motor atascado o sin encoder)

constexpr int SENTIDO_LEVANTAR = 1;       // +1 / -1: sentido de giro que ayuda a subir (invertir si empuja al reves)
constexpr bool SECUENCIA_DEMO = false;    // true = secuencia ida/regreso de prueba (NO usar con la asistencia)

constexpr uint32_t TIEMPO_GIRO_MAXIMO_MS = 2000;  // límite de seguridad
constexpr uint32_t TIEMPO_GIRO_INICIAL_MS = 200;  // 0.2 segundos

// ---- IMU MPU6050 (I2C) ----
// NO se usan los pines I2C por defecto (SDA=21) porque el 21 es IN2 del motor B
constexpr int PIN_SDA = 18;
constexpr int PIN_SCL = 19;
constexpr uint32_t I2C_FRECUENCIA_HZ = 100000;  // 100 kHz: más tolerante a cables largos y ruido

// ---- Tareas FreeRTOS (prioridad: mayor número = más prioritaria) ----
constexpr int CORE_TIEMPO_REAL = 1;  // control de motores
constexpr int CORE_AUXILIAR = 0;     // IMU, comandos

constexpr UBaseType_t PRIO_CONTROL_MOTORES = 4;  // lazo de 1 ms
constexpr UBaseType_t PRIO_IMU = 3;              // muestreo a 100 Hz
constexpr UBaseType_t PRIO_SECUENCIA = 2;
constexpr UBaseType_t PRIO_IMU_ESCRITURA = 2;    // escritura a flash, por debajo del muestreo
constexpr UBaseType_t PRIO_IMU_SERIE = 1;        // impresión de cada muestra en el monitor
constexpr UBaseType_t PRIO_COMANDOS = 1;
