/*
 * Automated Mini Conveyor Belt Speed Regulator
 * ESP32 + L298N + DC motor + optical (slotted-disc) sensor, closed-loop PID
 *
 * SIM_MODE 1 : no motor/sensor needed. A software motor model generates the
 *              optical pulses, so the full control loop runs in Wokwi.
 * SIM_MODE 0 : real hardware. Optical sensor output -> GPIO34 (interrupt).
 *
 * Controls
 *   Potentiometer (GPIO32) : target belt speed (0 .. MAX_SET_RPM)
 *   Button        (GPIO27) : toggle LOAD disturbance (SIM) / log marker (real)
 *
 * NOTE: save this file as sketch.ino (Wokwi) - not main.c
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SIM_MODE 1

// ---------- Pins ----------
#define PIN_ENA     14   // L298N ENA (PWM)
#define PIN_IN1     26   // L298N IN1
#define PIN_IN2     25   // L298N IN2
#define PIN_SENSOR  34   // optical sensor OUT (input only pin)
#define PIN_POT     32
#define PIN_BTN     27   // active LOW
#define OLED_SDA    21
#define OLED_SCL    22

// ---------- Mechanical ----------
#define SLOTS          40        // slots on the encoder disc (pulses / rev)
#define ROLLER_D_CM    2.0f      // belt roller diameter
#define MAX_SET_RPM    300.0f

// ---------- Control ----------
#define CTRL_DT_MS     100
#define WIN            5         // sliding window = 5 x 100 ms = 0.5 s
#define KP             0.35f
#define KI             1.2f
#define KD             0.0f
#define OUT_MIN        0
#define OUT_MAX        255
#define MIN_START_DUTY 55        // L298N + motor dead zone compensation
#define PWM_FREQ       5000
#define PWM_BITS       8

// ---------- SIM motor model (same constants as docs/sim_model.py) ----------
#define SIM_MAX_RPM    330.0f
#define SIM_DEADBAND   40
#define SIM_TAU_S      0.4f
#define SIM_LOAD_DROP  0.20f

static Adafruit_SSD1306 oled(128, 64, &Wire, -1);

static volatile uint32_t pulseCount = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static bool loadOn = false;

static float simRpm = 0, simPhase = 0;

static void IRAM_ATTR sensorISR() {
  portENTER_CRITICAL_ISR(&mux);
  pulseCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

static void motorForward(int duty) {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, duty);
}
static void motorStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, 0);
}

// Advance the software motor one control period and fake the optical pulses
static void simStep(int duty, float dt) {
  float target = 0;
  if (duty >= SIM_DEADBAND) {
    target = (duty - SIM_DEADBAND) / (255.0f - SIM_DEADBAND) * SIM_MAX_RPM;
    if (loadOn) target *= (1.0f - SIM_LOAD_DROP);
  }
  simRpm += (target - simRpm) * dt / SIM_TAU_S;
  simPhase += simRpm / 60.0f * dt * SLOTS;
  uint32_t p = (uint32_t)simPhase;
  simPhase -= p;
  portENTER_CRITICAL(&mux);
  pulseCount += p;
  portEXIT_CRITICAL(&mux);
}

static void drawOLED(float sp, float rpm, int duty) {
  float cms = rpm / 60.0f * PI * ROLLER_D_CM;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.print("CONVEYOR PID ");
  oled.print(SIM_MODE ? "[SIM]" : "[HW]");
  oled.setCursor(0, 14); oled.printf("Set : %5.0f RPM", sp);
  oled.setCursor(0, 24); oled.printf("Act : %5.0f RPM", rpm);
  oled.setCursor(0, 34); oled.printf("Belt: %5.1f cm/s", cms);
  oled.setCursor(0, 44); oled.printf("PWM : %3d/255", duty);
  oled.setCursor(0, 54); oled.print(loadOn ? "LOAD: ON" : "LOAD: OFF");
  oled.display();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_SENSOR, INPUT);
  analogReadResolution(12);

  ledcAttach(PIN_ENA, PWM_FREQ, PWM_BITS);   // ESP32 Arduino core 3.x API
  motorStop();

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found, continuing without display");
  }
#if !SIM_MODE
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), sensorISR, RISING);
#endif
  Serial.println("t_ms,setpoint_rpm,measured_rpm,pwm,load");
}

void loop() {
  static uint32_t nextTick = millis();
  static uint32_t hist[WIN] = {0};
  static int hi = 0;
  static float integ = 0, prevErr = 0;
  static int duty = 0;
  static bool lastBtn = HIGH;
  static uint32_t lastDraw = 0;
  static float rpmMeas = 0, sp = 0;

  if ((int32_t)(millis() - nextTick) < 0) return;
  nextTick += CTRL_DT_MS;
  const float dt = CTRL_DT_MS / 1000.0f;

  // button (edge)
  bool b = digitalRead(PIN_BTN);
  if (lastBtn == HIGH && b == LOW) {
    loadOn = !loadOn;
    Serial.println(loadOn ? "# LOAD ON" : "# LOAD OFF");
  }
  lastBtn = b;

  // setpoint from potentiometer (small dead zone at the bottom = stop)
  int raw = analogRead(PIN_POT);
  sp = raw * MAX_SET_RPM / 4095.0f;
  if (sp < 10) sp = 0;

#if SIM_MODE
  simStep(duty, dt);
#endif

  // measure speed: pulses in the last WIN periods
  portENTER_CRITICAL(&mux);
  uint32_t pc = pulseCount;
  pulseCount = 0;
  portEXIT_CRITICAL(&mux);
  hist[hi] = pc;
  hi = (hi + 1) % WIN;
  uint32_t sum = 0;
  for (int i = 0; i < WIN; i++) sum += hist[i];
  rpmMeas = (float)sum / SLOTS / (WIN * dt) * 60.0f;

  // PID with anti-windup
  if (sp <= 0) {
    integ = 0;
    duty = 0;
    motorStop();
  } else {
    float err = sp - rpmMeas;
    integ += err * dt;
    float u = KP * err + KI * integ + KD * (err - prevErr) / dt;
    float sat = constrain(u, (float)OUT_MIN, (float)OUT_MAX);
    if (u != sat) integ -= err * dt;       // stop integrating when saturated
    duty = (int)sat;
    if (duty > 0 && duty < MIN_START_DUTY) duty = MIN_START_DUTY;
    prevErr = err;
    motorForward(duty);
  }

  Serial.printf("%lu,%.1f,%.1f,%d,%d\n", millis(), sp, rpmMeas, duty, loadOn ? 1 : 0);

  if (millis() - lastDraw >= 250) {
    lastDraw = millis();
    drawOLED(sp, rpmMeas, duty);
  }
}
