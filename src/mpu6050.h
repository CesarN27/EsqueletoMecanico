// Driver mínimo del MPU6050 por I2C (sin dependencias). Solo lo deben usar la tarea del IMU
// o el diagnóstico con el registro pausado, nunca las dos a la vez.
#pragma once
#include <Arduino.h>

struct MuestraImu {
  float ax, ay, az;  // g
  float gx, gy, gz;  // grados/s
  float temp;        // °C
};

struct EstadisticasMpu {
  uint32_t lecturas = 0;
  uint32_t fallos = 0;
  uint32_t reconexiones = 0;
  uint8_t ultimoError = 0;  // ver mpuTextoError()
};

constexpr uint8_t REG_WHO_AM_I = 0x75;

void mpuAbrirBus();
void mpuCerrarBus();
void mpuRecuperarBus();  // libera un bus trabado (9 pulsos de reloj + STOP) y lo reabre

bool mpuPresente(uint8_t direccion);  // ¿alguien contesta (ACK) en esa dirección?
bool mpuIniciar();                    // detecta 0x68/0x69, despierta, configura y verifica
bool mpuLeer(MuestraImu& m);
bool mpuLeerReg(uint8_t reg, uint8_t& valor);

uint8_t mpuDireccion();
EstadisticasMpu& mpuEstadisticas();
const char* mpuTextoError(uint8_t codigo);
