#include "motores.h"

#include "config.h"
#include "log.h"

struct Motor {
  int pinPwm, pinIn1, pinIn2, pinEncoder, canalPwm;
  volatile long pos = 0;  // pulsos acumulados (con signo)
  volatile int dir = 0;   // sentido del último comando: +1 / -1
  volatile uint32_t ultimoUs = 0;
  volatile long objetivo = 0;
  volatile bool moviendo = false;
  volatile bool girando = false;  // PWM encendido (por secuencia, asistencia o destino)
  volatile uint32_t inicioMs = 0;

  Motor(int pwm, int in1, int in2, int enc, int canal)
      : pinPwm(pwm), pinIn1(in1), pinIn2(in2), pinEncoder(enc), canalPwm(canal) {}
};

static Motor motorA(25, 26, 27, 34, 0);
static Motor motorB(33, 32, 21, 35, 1);
static Motor* const motores[] = {&motorA, &motorB};

static volatile uint32_t tiempoGiroMs = TIEMPO_GIRO_INICIAL_MS;

// El encoder tiene un solo canal: el sentido se toma del comando de giro
static void IRAM_ATTR contarA() {
  uint32_t t = micros();
  if (t - motorA.ultimoUs > ANTIREBOTE_US) {
    motorA.ultimoUs = t;
    motorA.pos += motorA.dir;
  }
}

static void IRAM_ATTR contarB() {
  uint32_t t = micros();
  if (t - motorB.ultimoUs > ANTIREBOTE_US) {
    motorB.ultimoUs = t;
    motorB.pos += motorB.dir;
  }
}

static void iniciarMotor(Motor& m, void (*isr)()) {
  pinMode(m.pinIn1, OUTPUT);
  pinMode(m.pinIn2, OUTPUT);
  pinMode(m.pinEncoder, INPUT);
  ledcSetup(m.canalPwm, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(m.pinPwm, m.canalPwm);
  attachInterrupt(digitalPinToInterrupt(m.pinEncoder), isr, RISING);
}

static void frenar(Motor& m) {
  digitalWrite(m.pinIn1, HIGH);
  digitalWrite(m.pinIn2, HIGH);
  ledcWrite(m.canalPwm, 0);
  m.moviendo = false;
  m.girando = false;
}

static void girar(Motor& m, int sentido) {
  m.dir = sentido;
  digitalWrite(m.pinIn1, sentido > 0 ? HIGH : LOW);
  digitalWrite(m.pinIn2, sentido > 0 ? LOW : HIGH);
  ledcWrite(m.canalPwm, VELOCIDAD);
  m.girando = true;
}

// Detiene el motor al llegar al objetivo o por timeout
static void actualizar(Motor& m) {
  if (!m.moviendo) return;
  bool llego = (m.dir > 0 && m.pos >= m.objetivo) || (m.dir < 0 && m.pos <= m.objetivo);
  if (llego || millis() - m.inicioMs > TIMEOUT_MS) frenar(m);
}

// Lazo de control de 1 ms, el de mayor prioridad
static void tareaControl(void*) {
  TickType_t t0 = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(1));
    actualizar(motorA);
    actualizar(motorB);
  }
}

static void pausa(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Secuencia: ida durante tiempoGiroMs, pausa, regreso el mismo tiempo, pausa
static void tareaSecuencia(void*) {
  for (;;) {
    logPrintf("Ida: %lu ms\n", (unsigned long)tiempoGiroMs);
    girar(motorA, 1);
    girar(motorB, 1);
    pausa(tiempoGiroMs);  // <<< el motor gira este tiempo
    frenar(motorA);
    frenar(motorB);
    pausa(3000);  // pausa en la posición alcanzada

    logPrintf("Regreso a la posicion inicial\n");
    girar(motorA, -1);
    girar(motorB, -1);
    pausa(tiempoGiroMs);  // <<< mismo tiempo en sentido contrario
    frenar(motorA);
    frenar(motorB);
    logPrintf("Posicion A=%ld B=%ld pulsos\n", motorA.pos, motorB.pos);
    pausa(3000);  // pausa en la posición inicial
  }
}

void motoresIniciar() {
  pinMode(PIN_STBY, OUTPUT);
  iniciarMotor(motorA, contarA);
  iniciarMotor(motorB, contarB);
  frenar(motorA);
  frenar(motorB);
  digitalWrite(PIN_STBY, HIGH);

  xTaskCreatePinnedToCore(tareaControl, "ctrl_motores", 2048, nullptr, PRIO_CONTROL_MOTORES,
                          nullptr, CORE_TIEMPO_REAL);
  if (SECUENCIA_DEMO)
    xTaskCreatePinnedToCore(tareaSecuencia, "sec_motores", 3072, nullptr, PRIO_SECUENCIA, nullptr,
                            CORE_TIEMPO_REAL);
  logPrintf("Motores listos (posicion actual = 0)\n");
}

void motoresSetTiempoGiro(uint32_t ms) { tiempoGiroMs = min(ms, TIEMPO_GIRO_MAXIMO_MS); }

uint32_t motoresTiempoGiro() { return tiempoGiroMs; }

void motoresIrAGrados(int motor, float grados) {
  if (motor < 0 || motor > 1) return;
  Motor& m = *motores[motor];
  m.objetivo = lround(grados * PULSOS_POR_VUELTA / 360.0f);
  if (m.objetivo == m.pos) return;
  m.inicioMs = millis();
  girar(m, m.objetivo > m.pos ? 1 : -1);
  m.moviendo = true;
}

void motoresAsistir(bool activo) {
  if (activo) {
    girar(motorA, SENTIDO_LEVANTAR);
    girar(motorB, SENTIDO_LEVANTAR);
  } else {
    frenar(motorA);
    frenar(motorB);
  }
}

bool motoresGirando() { return motorA.girando || motorB.girando; }

long motoresPulsos(int motor) { return motores[motor == 1 ? 1 : 0]->pos; }
