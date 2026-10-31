#include "mpu6050.h"

#include <Wire.h>

#include "config.h"

constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_DATOS = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;

// Códigos 1..5 son los de Wire.endTransmission(); 6 y 7 son propios
constexpr uint8_t ERR_LECTURA_INCOMPLETA = 6;
constexpr uint8_t ERR_REGISTRO_NO_GUARDA = 7;

static uint8_t direccion = 0x68;
static bool busAbierto = false;
static EstadisticasMpu stats;

static void fallo(uint8_t codigo) {
  stats.fallos++;
  stats.ultimoError = codigo;
}

void mpuAbrirBus() {
  if (busAbierto) return;
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_FRECUENCIA_HZ);
  Wire.setTimeOut(15);  // ms; un bus trabado no debe bloquear mucho la tarea
  busAbierto = true;
}

void mpuCerrarBus() {
  if (!busAbierto) return;
  Wire.end();
  busAbierto = false;
}

void mpuRecuperarBus() {
  mpuCerrarBus();
  // Si un esclavo se quedó reteniendo SDA a medio byte, 9 pulsos de reloj lo hacen soltarla
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, HIGH);
  digitalWrite(PIN_SCL, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(10);
  }
  // Condición STOP
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SDA, HIGH);
  delayMicroseconds(10);

  mpuAbrirBus();
  stats.reconexiones++;
}

bool mpuPresente(uint8_t dir) {
  Wire.beginTransmission(dir);
  uint8_t err = Wire.endTransmission();
  if (err) stats.ultimoError = err;
  return err == 0;
}

static bool escribirReg(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(direccion);
  Wire.write(reg);
  Wire.write(valor);
  uint8_t err = Wire.endTransmission();
  if (err) stats.ultimoError = err;
  return err == 0;
}

bool mpuLeerReg(uint8_t reg, uint8_t& valor) {
  Wire.beginTransmission(direccion);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err) {
    stats.ultimoError = err;
    return false;
  }
  if (Wire.requestFrom((int)direccion, 1) != 1) {
    stats.ultimoError = ERR_LECTURA_INCOMPLETA;
    return false;
  }
  valor = Wire.read();
  return true;
}

bool mpuIniciar() {
  mpuAbrirBus();
  if (mpuPresente(0x68)) direccion = 0x68;
  else if (mpuPresente(0x69)) direccion = 0x69;
  else return false;

  // Sale de reposo con reloj PLL del giroscopio X, filtro ~44 Hz, ±250 °/s, ±2 g
  if (!escribirReg(REG_PWR_MGMT_1, 0x01)) return false;
  vTaskDelay(pdMS_TO_TICKS(50));
  if (!escribirReg(REG_CONFIG, 0x03) || !escribirReg(REG_GYRO_CONFIG, 0x00) ||
      !escribirReg(REG_ACCEL_CONFIG, 0x00))
    return false;

  uint8_t v;
  if (!mpuLeerReg(REG_CONFIG, v)) return false;
  if (v != 0x03) {
    stats.ultimoError = ERR_REGISTRO_NO_GUARDA;
    return false;
  }
  return true;
}

static int16_t leer16() {
  int16_t alto = Wire.read();
  int16_t bajo = Wire.read();
  return (alto << 8) | bajo;
}

bool mpuLeer(MuestraImu& m) {
  stats.lecturas++;
  Wire.beginTransmission(direccion);
  Wire.write(REG_DATOS);
  uint8_t err = Wire.endTransmission(false);
  if (err) {
    fallo(err);
    return false;
  }
  if (Wire.requestFrom((int)direccion, 14) != 14) {
    while (Wire.available()) Wire.read();
    fallo(ERR_LECTURA_INCOMPLETA);
    return false;
  }
  m.ax = leer16() / 16384.0f;  // rango ±2 g
  m.ay = leer16() / 16384.0f;
  m.az = leer16() / 16384.0f;
  m.temp = leer16() / 340.0f + 36.53f;
  m.gx = leer16() / 131.0f;  // rango ±250 °/s
  m.gy = leer16() / 131.0f;
  m.gz = leer16() / 131.0f;
  return true;
}

uint8_t mpuDireccion() { return direccion; }

EstadisticasMpu& mpuEstadisticas() { return stats; }

const char* mpuTextoError(uint8_t codigo) {
  switch (codigo) {
    case 0: return "sin error";
    case 1: return "dato demasiado largo";
    case 2: return "NACK en la direccion: el sensor no contesta";
    case 3: return "NACK en un dato";
    case 4: return "error de bus";
    case 5: return "timeout: bus trabado (SDA o SCL retenida)";
    case 6: return "lectura incompleta";
    case 7: return "el registro no guarda el valor escrito";
    default: return "error desconocido";
  }
}
