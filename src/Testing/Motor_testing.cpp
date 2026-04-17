#include <Arduino.h>
#include <Teensy_PWM.h>

// -----------------------------
// Your exact pinouts (unchanged)
// -----------------------------

// Motor Driver Pins
#define MOTOR0_PWM_A 4
#define MOTOR0_PWM_B 5
#define MOTOR1_PWM_A 10
#define MOTOR1_PWM_B 11
#define MOTOR2_PWM_A 24
#define MOTOR2_PWM_B 25
#define MOTOR3_PWM_A 28
#define MOTOR3_PWM_B 29
#define MOTOR4_PWM_A 15
#define MOTOR4_PWM_B 14
#define MOTOR5_PWM_A 37
#define MOTOR5_PWM_B 36

// Motor Driver control pins (as in your code)
#define PMODE  12
#define nFAULT 26
#define nSLEEP 27
#define vREF   33

// Arrays like your project
const int motorPWM_A[6] = {MOTOR0_PWM_A, MOTOR1_PWM_A, MOTOR2_PWM_A, MOTOR3_PWM_A, MOTOR4_PWM_A, MOTOR5_PWM_A};
const int motorPWM_B[6] = {MOTOR0_PWM_B, MOTOR1_PWM_B, MOTOR2_PWM_B, MOTOR3_PWM_B, MOTOR4_PWM_B, MOTOR5_PWM_B};

// -----------------------------
// Teensy_PWM setup (your values)
// -----------------------------
constexpr float MOTOR_PWM_FREQ_HZ = 20000.0f; // 20 kHz for DRV8874
constexpr float VREF_PWM_FREQ_HZ  = 20000.0f;

Teensy_PWM* motorPwmA[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
Teensy_PWM* motorPwmB[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
Teensy_PWM* vrefPwm       = nullptr;

// Mirrors your “last applied” logic
uint8_t lastPWM_A[6] = {0,0,0,0,0,0};
uint8_t lastPWM_B[6] = {0,0,0,0,0,0};
uint8_t lastVrefDuty = 0;

// -----------------------------
// Helpers (extracted from yours)
// -----------------------------
static inline uint16_t dutyByteToLevel16(uint8_t dutyByte) {
  return (uint32_t)dutyByte * 65535u / 255u;
}

static inline void setMotorDutyByte(Teensy_PWM* pwm, uint8_t dutyByte) {
  if (!pwm) return;
  const uint16_t level = dutyByteToLevel16(dutyByte);
  pwm->setPWM_manual((uint8_t)pwm->getPin(), level);
}

// vREF duty update using Teensy_PWM (0..255)
void setVrefDuty(int dutyByte) {
  dutyByte = constrain(dutyByte, 0, 255);

  if ((uint8_t)dutyByte == lastVrefDuty) return;
  lastVrefDuty = (uint8_t)dutyByte;

  if (!vrefPwm) return;

  const uint16_t level = dutyByteToLevel16((uint8_t)dutyByte);
  vrefPwm->setPWM_manual((uint8_t)vrefPwm->getPin(), level);
}

// Convenience: set motor i to coast-mode forward/reverse like your project
static inline void setDriveCoast(int index, int dir, uint8_t duty) {
  duty = constrain(duty, 0, 255);

  // Only touch hardware when changed (like your code)
  if (dir > 0) {
    if (duty != lastPWM_A[index]) { setMotorDutyByte(motorPwmA[index], duty); lastPWM_A[index] = duty; }
    if (0    != lastPWM_B[index]) { setMotorDutyByte(motorPwmB[index], 0);    lastPWM_B[index] = 0;    }
  } else if (dir < 0) {
    if (0    != lastPWM_A[index]) { setMotorDutyByte(motorPwmA[index], 0);    lastPWM_A[index] = 0;    }
    if (duty != lastPWM_B[index]) { setMotorDutyByte(motorPwmB[index], duty); lastPWM_B[index] = duty; }
  } else {
    if (0 != lastPWM_A[index]) { setMotorDutyByte(motorPwmA[index], 0); lastPWM_A[index] = 0; }
    if (0 != lastPWM_B[index]) { setMotorDutyByte(motorPwmB[index], 0); lastPWM_B[index] = 0; }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Pin modes (mirrors your setup)
  for (int i = 0; i < 6; i++) {
    pinMode(motorPWM_A[i], OUTPUT);
    pinMode(motorPWM_B[i], OUTPUT);
  }

  pinMode(PMODE, OUTPUT);
  pinMode(vREF, OUTPUT);
  pinMode(nFAULT, INPUT_PULLUP);
  pinMode(nSLEEP, OUTPUT);

  // Enable driver pins in a sane way for a bench test
  digitalWrite(PMODE, HIGH);
  digitalWrite(nSLEEP, HIGH);

  // ------------------------------
  // Initialize Teensy_PWM outputs (motors)
  // ------------------------------
  for (int i = 0; i < 6; i++) {
    motorPwmA[i] = new Teensy_PWM(motorPWM_A[i], MOTOR_PWM_FREQ_HZ, 0.0f);
    motorPwmB[i] = new Teensy_PWM(motorPWM_B[i], MOTOR_PWM_FREQ_HZ, 0.0f);

    if (!motorPwmA[i] || !motorPwmB[i]) {
      Serial.println("PWM allocation failed!");
      while (1) {}
    }

    motorPwmA[i]->setPWM();
    motorPwmB[i]->setPWM();
  }

  // ------------------------------
  // Initialize Teensy_PWM output (vREF)
  // ------------------------------
  vrefPwm = new Teensy_PWM(vREF, VREF_PWM_FREQ_HZ, 0.0f);
  if (!vrefPwm) {
    Serial.println("vREF PWM allocation failed!");
    while (1) {}
  }
  vrefPwm->setPWM();

  // Set full-scale vREF (like NORMAL_VREF_DUTY = 255)
  setVrefDuty(255);

  Serial.println("Teensy_PWM minimal sketch initialized.");
}

void loop() {
  // Demo: ramp Motor 0 forward, then reverse, then stop.
  const int m = 0;

  Serial.println("Forward ramp (A up, B=0)");
  for (int d = 0; d <= 255; d += 5) {
    setDriveCoast(m, +1, (uint8_t)d);
    delay(30);
  }

  Serial.println("Stop");
  setDriveCoast(m, 0, 0);
  delay(300);

  Serial.println("Reverse ramp (B up, A=0)");
  for (int d = 0; d <= 255; d += 5) {
    setDriveCoast(m, -1, (uint8_t)d);
    delay(30);
  }

  Serial.println("Stop");
  setDriveCoast(m, 0, 0);
  delay(800);
}