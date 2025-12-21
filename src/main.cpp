// Version: 2025-07-20
// Description: Updated for CAN-based control — joystick + button data now received over CAN
// Update (2025-12-13): Migrated motor PWM outputs to Teensy_PWM (khoih-prog)
// Update (2025-12-19): vREF migrated BACK to Teensy_PWM for unified PWM control
// Update (2025-12-19): REMOVED nFAULT ISR + related globals/prints for debug isolation

// #include <PWMServo.h>  // TEMP DISABLED for PWM timer conflict testing
#include <Arduino.h>
#include <Encoder.h>
#include <FlexCAN_T4.h>
#include <Teensy_PWM.h>

// ----------- CAN Message Map ----------- //

// Brain -> Hand
constexpr uint32_t CAN_ID_CONFIG_CMD      = 0x100;  // requestedConfig
constexpr uint32_t CAN_ID_HOMING_CMD      = 0x120;  // homing start
constexpr uint32_t CAN_ID_PWM_A_CMD       = 0x200;  // PWM_A[0..5]
constexpr uint32_t CAN_ID_PWM_B_CMD       = 0x201;  // PWM_B[0..5]

// Hand -> Brain
constexpr uint32_t CAN_ID_ENCODER_BASE    = 0x110;  // + motor index
constexpr uint32_t CAN_ID_ROM_BASE        = 0x130;  // + motor index

// ----------- Encoder Pins (Hardware Quadrature) ----------- //
#define ENC0_A 0
#define ENC0_B 1
#define ENC1_A 2
#define ENC1_B 3
#define ENC2_A 6
#define ENC2_B 7
#define ENC3_A 8
#define ENC3_B 9
#define ENC4_A 22
#define ENC4_B 23
#define ENC5_A 34
#define ENC5_B 35

Encoder encoder0(ENC0_A, ENC0_B);
Encoder encoder1(ENC1_A, ENC1_B);
Encoder encoder2(ENC2_A, ENC2_B);
Encoder encoder3(ENC3_A, ENC3_B);
Encoder encoder4(ENC4_A, ENC4_B);
Encoder encoder5(ENC5_A, ENC5_B);

// ----------- Motor Driver Pins ----------- //
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

// ----------- Current Sense Pins ----------- //
#define MOTOR0_SEN 21
#define MOTOR1_SEN 20
#define MOTOR2_SEN 19
#define MOTOR3_SEN 18
#define MOTOR4_SEN 17
#define MOTOR5_SEN 16

// ----------- Servo Pins ----------- //
#define SERVO0 33

// ----------- Motor Driver Pins Declaration ----------- //
#define PMODE 12
#define nFAULT 26
#define nSLEEP 27
#define vREF 13

// ----------- CAN Declaration ----------- //
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

volatile uint8_t requestedConfig = 0;
volatile bool configChangeRequested = false;

// Homing request flag (set by CAN, executed in loop)
volatile bool homingRequested = false;
volatile uint8_t homingIndexRequested = 0;

// ----------- Global ROM limits for Motor Limits ----------- //
long lowROM[6]  = {-100000, -100000, -100000, -100000, -100000, -100000};
long highROM[6] = {100000, 100000, 100000, 100000, 100000, 100000};

// Homing timeouts [ms]
const unsigned long HOMING_HIGH_TIMEOUT_MS = 1000;  // high end search
const unsigned long HOMING_LOW_TIMEOUT_MS  = 1000;  // low end search

Encoder* encoders[6] = { &encoder0, &encoder1, &encoder2, &encoder3, &encoder4, &encoder5 };
const int motorPWM_A[6] = {MOTOR0_PWM_A, MOTOR1_PWM_A, MOTOR2_PWM_A, MOTOR3_PWM_A, MOTOR4_PWM_A, MOTOR5_PWM_A};
const int motorPWM_B[6] = {MOTOR0_PWM_B, MOTOR1_PWM_B, MOTOR2_PWM_B, MOTOR3_PWM_B, MOTOR4_PWM_B, MOTOR5_PWM_B};
const int motorSEN[6]   = {MOTOR0_SEN, MOTOR1_SEN, MOTOR2_SEN, MOTOR3_SEN, MOTOR4_SEN, MOTOR5_SEN};

// Incoming commands from brain (0..255 like old analogWrite)
uint8_t PWM_A[6] = {0, 0, 0, 0, 0, 0};
uint8_t PWM_B[6] = {0, 0, 0, 0, 0, 0};

// Last-applied PWM values (so we only touch hardware when something changes)
uint8_t lastPWM_A[6] = {0, 0, 0, 0, 0, 0};
uint8_t lastPWM_B[6] = {0, 0, 0, 0, 0, 0};

// Last-applied vREF duty byte
uint8_t lastVrefDuty = 0;

// Homing vREF voltage [PWM]
const int homingVoltage[6] = {50, 127, 127, 127, 127, 127};

// Normal (full-scale) vREF duty for regular operation
const int NORMAL_VREF_DUTY = 50;  // 100% duty (0..255)

// Homing speeds [0-100] P will be scaled later
const int homingSpeeds[6] = {90, 50, 50, 50, 50, 50};

// Pullback distances after stall [encoder counts]
const int pullbackCounts[6] = {10, 10, 10, 10, 10, 10};

// PWMServo servo0;  // TEMP DISABLED for PWM timer conflict testing

// =============================
// Teensy_PWM setup
// =============================
constexpr float MOTOR_PWM_FREQ_HZ = 20000.0f; // 20 kHz recommended for DRV8874
constexpr float VREF_PWM_FREQ_HZ  = 20000.0f; // keep vREF on same freq unless you have a reason not to

// One Teensy_PWM object per motor PWM pin
Teensy_PWM* motorPwmA[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
Teensy_PWM* motorPwmB[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

// vREF PWM object
Teensy_PWM* vrefPwm = nullptr;

// ----------- Function declarations -----------
void handleCAN(const CAN_message_t &msg);
void sendEncoderPositions();
void sendRangeOfMotion();
void nextState();
void handleState();
void printEncoderData();
void printMotorDiagnostics();
void controlMotor(int index);
void homeMotor(int index);
void stopMotor(int index);
void driveMotorHoming(int index, int direction);

static void loadPwmArray(uint8_t *target, const CAN_message_t &msg);

// ----------- Helper forward declarations -----------
static inline uint16_t dutyByteToLevel16(uint8_t dutyByte);
static inline void setMotorDutyByte(Teensy_PWM* pwm, uint8_t dutyByte);
void setVrefDuty(int dutyByte);

// Declaration for finger configurations driven by servos
enum HandState { CONFIG_1, CONFIG_2, CONFIG_3, CONFIG_4 };
HandState currentState = CONFIG_1;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  noInterrupts();

  // Motor PWM pins
  for (int i = 0; i < 6; i++) {
    pinMode(motorPWM_A[i], OUTPUT);
    pinMode(motorPWM_B[i], OUTPUT);
  }

  pinMode(PMODE, OUTPUT);
  pinMode(vREF, OUTPUT);
  pinMode(nFAULT, INPUT_PULLUP);   // external pull-up
  pinMode(nSLEEP, OUTPUT);

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
  // NOTE: Teensy_PWM takes dutyCycle in percent for the constructor.
  vrefPwm = new Teensy_PWM(vREF, VREF_PWM_FREQ_HZ, 0.0f);
  if (!vrefPwm) {
    Serial.println("vREF PWM allocation failed!");
    while (1) {}
  }
  vrefPwm->setPWM();
  setVrefDuty(NORMAL_VREF_DUTY);

  // servo0.attach(SERVO0);  // TEMP DISABLED for PWM timer conflict testing

  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.enableMBInterrupts();
  Can3.onReceive(handleCAN);

  Serial.println("System Initialized");

  digitalWrite(PMODE, HIGH);
  delay(50);
  digitalWrite(nSLEEP, HIGH);

  interrupts();

  sendRangeOfMotion();
  handleState();
}

void loop() {
  if (homingRequested) {
    homingRequested = false;
    Serial.println("Homing request received via CAN (deferred to loop)");
    homeMotor(homingIndexRequested);
    sendRangeOfMotion();
  }

  Can3.events();

  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100) {
    sendEncoderPositions();
    lastSend = millis();
  }

  for (int i = 0; i < 6; i++) {
    controlMotor(i);
  }

  if (configChangeRequested) {
    configChangeRequested = false;
    handleState();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    printEncoderData();
    printMotorDiagnostics();

    Serial.print("nFAULT pin: ");
    Serial.println(digitalRead(nFAULT) ? "HIGH" : "LOW");

    Serial.println();
  }
}

void handleCAN(const CAN_message_t &msg) {
  switch (msg.id) {
    case CAN_ID_PWM_A_CMD: {
      if (msg.len >= 6) loadPwmArray(PWM_A, msg);
      break;
    }

    case CAN_ID_PWM_B_CMD: {
      if (msg.len >= 6) loadPwmArray(PWM_B, msg);
      break;
    }

    case CAN_ID_CONFIG_CMD: {
      if (msg.len >= 1) {
        requestedConfig = msg.buf[0];
        configChangeRequested = true;
      }
      break;
    }

    case CAN_ID_HOMING_CMD: {
      if (msg.len == 1 && msg.buf[0] == 1) {
        homingIndexRequested = 0; // for now: motor 0
        homingRequested = true;
      }
      break;
    }

    default:
      break;
  }
}

void nextState() {
  currentState = static_cast<HandState>((currentState + 1) % 4);
  Serial.print("Transitioned to state: ");
  Serial.println(currentState);
  handleState();
}

void handleState() {
  HandState newState;

  switch (requestedConfig) {
    case 0: newState = CONFIG_1; break;
    case 1: newState = CONFIG_2; break;
    case 2: newState = CONFIG_3; break;
    case 3: newState = CONFIG_4; break;
    default:
      Serial.print("Ignoring invalid requestedConfig: ");
      Serial.println(requestedConfig);
      return;
  }

  if (newState == currentState) return;

  currentState = newState;

  Serial.print("Transitioned to state: ");
  Serial.println(currentState);

  switch (currentState) {
    case CONFIG_1:
      Serial.println("State 1: One-sided Grip");
      break;

    case CONFIG_2:
      Serial.println("State 2: Pinch Grip");
      break;

    case CONFIG_3:
      Serial.println("State 3: Claw Grip");
      break;

    case CONFIG_4:
      Serial.println("State 4: Coffee Cup Grip");
      break;
  }
}

void controlMotor(int index) {
  const uint8_t a = PWM_A[index];
  const uint8_t b = PWM_B[index];

  if (a != lastPWM_A[index]) {
    setMotorDutyByte(motorPwmA[index], a);
    lastPWM_A[index] = a;
  }
  if (b != lastPWM_B[index]) {
    setMotorDutyByte(motorPwmB[index], b);
    lastPWM_B[index] = b;
  }
}

void stopMotor(int index) {
  PWM_A[index] = 0;
  PWM_B[index] = 0;
  controlMotor(index);
}

// ----------- Helpers -----------
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

void printEncoderData() {
  Serial.print("Enc0: "); Serial.print(encoder0.read()); Serial.print("\t");
  Serial.print("Enc1: "); Serial.print(encoder1.read()); Serial.print("\t");
  Serial.print("Enc2: "); Serial.print(encoder2.read()); Serial.print("\t");
  Serial.print("Enc3: "); Serial.print(encoder3.read()); Serial.print("\t");
  Serial.print("Enc4: "); Serial.print(encoder4.read()); Serial.print("\t");
  Serial.print("Enc5: "); Serial.println(encoder5.read());
}

void printMotorDiagnostics() {
  constexpr float VADC = 3.3f;
  constexpr float ADC_MAX = 4095.0f;
  constexpr float RIPROPI_OHMS = 5100.0f;
  constexpr float AIPROPI_A_PER_A = 450e-6f;

  for (int i = 0; i < 6; i++) {
    int raw = analogRead(motorSEN[i]);
    float vSense = raw * (VADC / ADC_MAX);
    float iMotor = vSense / (AIPROPI_A_PER_A * RIPROPI_OHMS);

    Serial.print("M"); Serial.print(i);
    Serial.print(": PWM_A="); Serial.print(PWM_A[i]);
    Serial.print(" PWM_B="); Serial.print(PWM_B[i]);
    Serial.print("  ADC="); Serial.print(raw);
    Serial.print("  V="); Serial.print(vSense, 3);
    Serial.print("  I="); Serial.print(iMotor, 3);
    Serial.println(" A");
  }

  Serial.print("vREF duty byte: ");
  Serial.println(lastVrefDuty);
}

static void loadPwmArray(uint8_t *target, const CAN_message_t &msg) {
  const uint8_t count = min<uint8_t>(6, msg.len);
  for (uint8_t i = 0; i < count; ++i) target[i] = msg.buf[i];
}

void sendEncoderPositions() {
  for (int i = 0; i < 6; i++) {
    CAN_message_t msg;
    msg.id = CAN_ID_ENCODER_BASE + i;
    msg.len = 4;
    long pos = encoders[i]->read();

    msg.buf[0] = (pos >> 24) & 0xFF;
    msg.buf[1] = (pos >> 16) & 0xFF;
    msg.buf[2] = (pos >> 8) & 0xFF;
    msg.buf[3] = pos & 0xFF;

    Can3.write(msg);
  }
}

void sendRangeOfMotion() {
  for (int i = 0; i < 6; i++) {
    CAN_message_t msg;
    msg.id = CAN_ID_ROM_BASE + i;
    msg.len = 8;

    msg.buf[0] = (lowROM[i] >> 24) & 0xFF;
    msg.buf[1] = (lowROM[i] >> 16) & 0xFF;
    msg.buf[2] = (lowROM[i] >> 8) & 0xFF;
    msg.buf[3] = lowROM[i] & 0xFF;

    msg.buf[4] = (highROM[i] >> 24) & 0xFF;
    msg.buf[5] = (highROM[i] >> 16) & 0xFF;
    msg.buf[6] = (highROM[i] >> 8) & 0xFF;
    msg.buf[7] = highROM[i] & 0xFF;

    Can3.write(msg);
  }
}

// direction: +1 or -1
void driveMotorHoming(int index, int direction) {
  int duty = map(homingSpeeds[index], 0, 100, 0, 255);
  duty = constrain(duty, 0, 255);

  Serial.print("Current Homing Speed ");
  Serial.println(duty);

  if (direction > 0) {
    PWM_A[index] = duty;
    PWM_B[index] = 0;
  } else if (direction < 0) {
    PWM_A[index] = 0;
    PWM_B[index] = duty;
  } else {
    PWM_A[index] = 0;
    PWM_B[index] = 0;
  }

  controlMotor(index);
}

void homeMotor(int index) {
  Serial.print("Simple homing motor ");
  Serial.println(index);

  int nf = digitalRead(nFAULT);
  if (nf == LOW) {
    Serial.println("homeMotor(): nFAULT is LOW before start. Attempting to clear via nSLEEP toggle...");

    digitalWrite(nSLEEP, LOW);
    delay(5);
    digitalWrite(nSLEEP, HIGH);
    delay(5);

    nf = digitalRead(nFAULT);
    Serial.print("After nSLEEP toggle, nFAULT = ");
    Serial.println(nf == HIGH ? "HIGH" : "LOW");

    if (nf == LOW) {
      Serial.println("Aborting homing: nFAULT still LOW (driver still in fault). ");
      setVrefDuty(NORMAL_VREF_DUTY);
      return;
    }
  }

  // Set homing VREF for this motor
  setVrefDuty(homingVoltage[index]);
  
  // Dead-simple homing debug step: drive until nFAULT asserts low
  driveMotorHoming(index, +1);
  delay(500);
  unsigned long lastWaitPrint = 0;
  while (digitalRead(nFAULT) == HIGH) {
    if (millis() - lastWaitPrint >= 250) {
      lastWaitPrint = millis();
      Serial.println("[HOMING DEBUG] Waiting for nFAULT... pin=HIGH");
    }
    delay(1);
  }

  stopMotor(index);
  Serial.println("[HOMING DEBUG] nFAULT went LOW! Stopping motor and exiting debug homing step.");

  setVrefDuty(NORMAL_VREF_DUTY);
}
