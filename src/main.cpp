// Version: 2025-07-20
// Description: Updated for CAN-based control — joystick + button data now received over CAN
// Update (2025-12-13): Migrated motor PWM outputs to Teensy_PWM (khoih-prog)
// Update (2025-12-19): vREF migrated BACK to Teensy_PWM for unified PWM control
// Update (2025-12-19): REMOVED nFAULT ISR + related globals/prints for debug isolation
// Update (2026-01-03): Added queued homing sequence (Motor 0 -> Motor 1)

#include <Arduino.h>
#include <Encoder.h>
#include <FlexCAN_T4.h>
#include <Teensy_PWM.h>
#include <IntervalTimer.h>

// ----------- CAN Message Map ----------- //

// Brain -> Hand
constexpr uint32_t CAN_ID_HOMING_CMD      = 0x120;  // homing start
constexpr uint32_t CAN_ID_SET_SERVO_POS   = 0x220;  // servo0,servo1 angle targets (int32 LE)

// NEW: Brain -> Hand (absolute targets, int32 little-endian)
constexpr uint32_t CAN_ID_SET_POS_01      = 0x210;  // pos0,pos1 (int32 LE)
constexpr uint32_t CAN_ID_SET_POS_23      = 0x211;  // pos2,pos3
constexpr uint32_t CAN_ID_SET_POS_45      = 0x212;  // pos4,pos5

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
#define SERVO0 40
#define SERVO1 41

// ----------- Servo ISR PWM ----------- //
constexpr uint32_t SERVO_FRAME_US = 20000; // 20 ms = 50 Hz
constexpr uint32_t SERVO_MIN_PULSE_US = 500;
constexpr uint32_t SERVO_MAX_PULSE_US = 2500;
constexpr float SERVO_RANGE_DEG = 270.0f;

IntervalTimer servoTimer0;
IntervalTimer servoTimer1;

volatile uint32_t servoPulseWidthUs0 = 1500;
volatile uint32_t servoPulseWidthUs1 = 1500;
volatile bool servoPulseHigh0 = false;
volatile bool servoPulseHigh1 = false;

// ----------- Motor Driver Pins Declaration ----------- //
#define PMODE 12
#define nFAULT 26
#define nSLEEP 27
#define vREF 33

// ----------- CAN Declaration ----------- //
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// Homing request flag (set by CAN, executed in loop)
volatile bool homingRequested = false;

volatile bool homingActive = false;

// Which motors to home when a homing command is received.
// All listed motors run through the non-blocking 6-step homing state machine.
constexpr int HOMING_SEQUENCE[] = {0, 2, 4};
constexpr int HOMING_SEQUENCE_LEN = (int)(sizeof(HOMING_SEQUENCE) / sizeof(HOMING_SEQUENCE[0]));

// ----------- Global ROM limits for Motor Limits ----------- //
long lowROM[6]  = {-100000, -100000, -100000, -100000, -100000, -100000};
long highROM[6] = {100000, 100000, 100000, 100000, 100000, 100000};


Encoder* encoders[6] = { &encoder0, &encoder1, &encoder2, &encoder3, &encoder4, &encoder5 };
const int motorPWM_A[6] = {MOTOR0_PWM_A, MOTOR1_PWM_A, MOTOR2_PWM_A, MOTOR3_PWM_A, MOTOR4_PWM_A, MOTOR5_PWM_A};
const int motorPWM_B[6] = {MOTOR0_PWM_B, MOTOR1_PWM_B, MOTOR2_PWM_B, MOTOR3_PWM_B, MOTOR4_PWM_B, MOTOR5_PWM_B};
const int motorSEN[6]   = {MOTOR0_SEN, MOTOR1_SEN, MOTOR2_SEN, MOTOR3_SEN, MOTOR4_SEN, MOTOR5_SEN};

// NEW: absolute position targets received from Brain (raw, unclamped for now)
volatile int32_t posTarget[6] = {0,0,0,0,0,0};

// Servo angle targets received from Brain. Values are clamped in setServoAngle().
volatile int32_t servoTargetDeg[2] = {135, 135};
volatile bool servoTargetsUpdated = false;

// NEW: flags for safe printing outside ISR
volatile bool posTargetsUpdated = false;
elapsedMillis posTargetsUpdatedAgeMs;  // time since last update
volatile bool ignoreCanPosTargets = false;
elapsedMillis ignoreCanPosTargetsAgeMs;
static constexpr uint32_t HOMING_CAN_POS_HOLDOFF_MS = 10000;


// Incoming commands from brain (0..255 like old analogWrite)
uint8_t PWM_A[6] = {0, 0, 0, 0, 0, 0};
uint8_t PWM_B[6] = {0, 0, 0, 0, 0, 0};

// Last-applied PWM values (so we only touch hardware when something changes)
uint8_t lastPWM_A[6] = {0, 0, 0, 0, 0, 0};
uint8_t lastPWM_B[6] = {0, 0, 0, 0, 0, 0};

// Last-applied vREF duty byte
uint8_t lastVrefDuty = 0;


// =============================
// Position Controller (P now; PI/PID ready)
// =============================
enum ControlMode_t { MODE_POS = 0, MODE_PWM = 1 };
volatile ControlMode_t ControlMode[6] = { MODE_POS, MODE_POS, MODE_POS, MODE_POS, MODE_POS, MODE_POS };

// Per-motor gains (start with small Kp; Ki/Kd = 0 for now)
float Kp[6] = { 0.15f, 0.050f, 0.15f, 0.050f, 0.15f, 0.050f };
float Ki[6] = { 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f };
float Kd[6] = { 0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f   };

// Controller memory (for PI/PID later)
float integ[6] = {0,0,0,0,0,0};
int32_t prevErr[6] = {0,0,0,0,0,0};

// Output shaping
int32_t posDeadbandCounts[6] = { 10, 10, 10, 10, 10, 10 };   // within this, command 0
uint8_t posMaxDuty[6]        = { 255, 255, 255, 255, 255, 255};  // clamp output
uint8_t posMinDuty[6]        = { 50, 115, 50, 115, 50, 115 };   // overcome static friction (0 disables)

// =============================
// Homing Constants Setup
// =============================

// Stall detection thresholds [Amps]
// NOTE: Drive–Brake tends to produce higher average IPROPI/current than Drive–Coast at the same mechanical load.
// So we allow separate thresholds for BLUNT (drive–coast) vs PRECISE (drive–brake).
float homingStallA_blunt[6]   = { 0.20f, 0.30f, 0.20f, 0.15f, 0.20f, 0.15f };
float homingStallA_precise[6] = { 0.30f, 0.40f, 0.30f, 0.20f, 0.30f, 0.20f };

// Ignore current sensing for a short time after entering drive–brake to avoid immediate false "stall" due to braking current spikes.
uint32_t homingBrakeBlankMs[6] = { 100, 100, 100, 100, 100, 100 };

// Stall debounce time [ms]
uint32_t homingStallDebounceMs[6] = { 30, 30, 30, 30, 30, 30 };

// Current sample period during homing [ms]
uint32_t homingSampleMs[6] = { 5, 5, 5, 5, 5, 5 };

// Unstick phase
uint8_t  homingUnstickDuty[6]      = { 200, 200, 200, 200, 200, 200 };
uint32_t homingUnstickMs[6]        = { 3000, 3000, 3000, 3000, 3000, 3000 };
int      homingUnstickMinCounts[6] = { 100, 100, 100, 100, 100, 100 };

// Homing timeouts [ms] (Per motor)
// Blunt (drive–coast) search windows for LOW and HIGH ends.
uint32_t homingBluntTimeoutLowMs[6]  = { 12000, 20000, 12000, 20000, 12000, 20000 };
uint32_t homingBluntTimeoutHighMs[6] = { 12000, 20000, 12000, 20000, 12000, 20000 };


// Blunt seek (drive–coast)
uint8_t homingBluntDuty[6] = { 140, 160, 140, 160, 140, 160 };

// Backoff distances [encoder counts]
int homingBluntBackoffCounts[6] = { 3000, 5000, 3000, 5000, 3000, 5000 };
int homingTouchBackoffCounts[6] = { 3000, 9000, 3000, 9000, 3000, 9000 };

// Backoff timeouts [ms]
// Motors 1/3/5 use larger backoff distances and need longer windows to fully clear the stop.
uint32_t homingBluntBackoffTimeoutMs[6] = { 2000, 2500, 2000, 2500, 2000, 2500 };
uint32_t homingTouchBackoffTimeoutMs[6] = { 2000, 4500, 2000, 4500, 2000, 4500 };
uint8_t homingTouchBackoffDuty[6] = { 250, 255, 250, 255, 250, 255 };

// Precise touch-off (drive–brake)
uint8_t  homingPreciseDuty[6]      = { 140, 200, 140, 200, 140, 200 };
uint32_t homingPreciseTimeoutMs[6] = { 3200, 3200, 3200, 3200, 3200, 3200 };
uint8_t  homingTouchRepeats[6]     = { 3, 3, 3, 3, 3, 3 };

// Midpoint move
uint32_t homingMidTimeoutMs[6]   = { 5000, 5000, 5000, 5000, 5000, 5000 };
int      homingMidTolCounts[6]   = { 200, 200, 200, 200, 200, 200 };

// Homing vREF voltage [PWM]
const int homingVoltage[6] = {255, 255, 255, 255, 255, 255};

// Normal (full-scale) vREF duty for regular operation
const int NORMAL_VREF_DUTY = 255;  // 100% duty (0..255)

//Buys time for Cap to settle.
static constexpr uint32_t VREF_SETTLE_MS   = 25;
static constexpr uint32_t HOMING_TOUCH_SETTLE_MS = 25;

constexpr int HOMING_MAX_TOUCH_SAMPLES = 5;

enum HomingPhase {
  HOMING_IDLE = 0,
  HOMING_VREF_SETTLE,
  HOMING_STEP1_UNSTICK_FWD,
  HOMING_STEP1_UNSTICK_REV,
  HOMING_STEP2_BLUNT_LOW,
  HOMING_STEP2_BACKOFF_LOW,
  HOMING_STEP3_PRECISE_LOW_DRIVE,
  HOMING_STEP3_PRECISE_LOW_BACKOFF,
  HOMING_STEP3_PRECISE_LOW_SETTLE,
  HOMING_STEP4_BLUNT_HIGH,
  HOMING_STEP4_BACKOFF_HIGH,
  HOMING_STEP5_PRECISE_HIGH_DRIVE,
  HOMING_STEP5_PRECISE_HIGH_BACKOFF,
  HOMING_STEP5_PRECISE_HIGH_SETTLE,
  HOMING_STEP6_MOVE_MID
};

struct HomingState {
  bool active = false;
  HomingPhase phase = HOMING_IDLE;
  uint32_t phaseStartMs = 0;
  uint32_t lastSampleMs = 0;
  uint32_t aboveStartMs = 0;
  long phaseStartPos = 0;
  uint8_t touchRepeatTarget = 0;
  uint8_t touchRepeatIndex = 0;
  long lowSamples[HOMING_MAX_TOUCH_SAMPLES] = {0};
  long highSamples[HOMING_MAX_TOUCH_SAMPLES] = {0};
  long lowEnd = 0;
  long highEnd = 0;
};

HomingState homingState[6];

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
void printEncoderData();
void printMotorDiagnostics();
void printPositionControllerDiagnostics();
void controlMotor(int index);
void startHomingMotor(int index);
void updateHomingMotor(int index);
void beginHomingSequence();
void updateHomingSequence();
void stopMotor(int index);

// ----------- Helper forward declarations -----------
void servoISR0();
void servoISR1();
void setServoPulseUs(uint8_t servoIndex, uint32_t us);
void setServoAngle(uint8_t servoIndex, float deg);
void applyServoTargets();
static inline uint16_t dutyByteToLevel16(uint8_t dutyByte);
static inline void setMotorDutyByte(Teensy_PWM* pwm, uint8_t dutyByte);
void setVrefDuty(int dutyByte);
static inline float readMotorCurrentA(int index);
static inline void setDriveCoast(int index, int dir, uint8_t duty);
static inline void setDriveBrake(int index, int dir, uint8_t driveDuty);
static inline int32_t unpack_i32_le(const uint8_t *b);
static inline void pack_i32_le(uint8_t *b, int32_t v);
static inline void enterPosModeHold(int index);
void updatePositionController(int index, float dt_s);

static void refreshHomingVref();
static void finishHomingMotor(int index, bool success, const char *message);
static void enterHomingPhase(int index, HomingPhase phase);
static bool homingMotionReached(int index, int dir, int minDeltaCounts);
static int homingTouchRepeatsClamped(int index);
static bool homingStallDetectedNonBlocking(int index,
                                           float stallA,
                                           uint32_t debounceMs,
                                           uint32_t sampleMs,
                                           uint32_t maxMs,
                                           uint32_t blankMs = 0);

static inline long median3(long a, long b, long c);
static long trimmedMean(const long *samples, int n);

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

  // Servo outputs (active-low pulse timing to match existing servo ISR behavior)
  pinMode(SERVO0, OUTPUT);
  pinMode(SERVO1, OUTPUT);
  digitalWriteFast(SERVO0, HIGH);
  digitalWriteFast(SERVO1, HIGH);

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


  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.enableMBInterrupts();
  Can3.onReceive(handleCAN);

  Serial.println("System Initialized");

  digitalWrite(PMODE, HIGH);
  delay(50);
  digitalWrite(nSLEEP, HIGH);

  // Kick off each servo ISR; each timer maintains a 50 Hz frame.
  servoTimer0.begin(servoISR0, 50);
  servoTimer1.begin(servoISR1, 50);

  interrupts();

  sendRangeOfMotion();
  applyServoTargets();
}

void loop() {
  if (homingRequested) {
    homingRequested = false;
    beginHomingSequence();
  }

  Can3.events();
  updateHomingSequence();

  if (ignoreCanPosTargets && !homingActive && ignoreCanPosTargetsAgeMs >= HOMING_CAN_POS_HOLDOFF_MS) {
    ignoreCanPosTargets = false;
    Serial.println("CAN position target updates resumed");
  }

  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100) {
    sendEncoderPositions();
    lastSend = millis();
  }

// Run POS controller at fixed rate (example: 5ms = 200 Hz)
static elapsedMicros posCtrlTimerUs;
static constexpr uint32_t POS_CTRL_PERIOD_US = 5000; // 5,000 us
if (posCtrlTimerUs >= POS_CTRL_PERIOD_US) {
  posCtrlTimerUs -= POS_CTRL_PERIOD_US; // stable timing
  const float dt_s = POS_CTRL_PERIOD_US * 1e-6f;

  for (int i = 0; i < 6; i++) {
    if (ControlMode[i] == MODE_POS) {
      updatePositionController(i, dt_s);
    }
  }
}

// Existing PWM application (only matters in MODE_PWM)
for (int i = 0; i < 6; i++) {
  if (ControlMode[i] == MODE_PWM) {
    controlMotor(i);
  }
}

  if (servoTargetsUpdated) {
    noInterrupts();
    servoTargetsUpdated = false;
    interrupts();
    applyServoTargets();
  }

  static unsigned long lastPrint = 0;
  if (!homingActive && millis() - lastPrint >= 100) {
    lastPrint = millis();

    printEncoderData();
    printMotorDiagnostics();
    printPositionControllerDiagnostics();

    Serial.print("nFAULT pin: ");
    Serial.println(digitalRead(nFAULT) ? "HIGH" : "LOW");

    Serial.println();
  }
}

void handleCAN(const CAN_message_t &msg) {
  switch (msg.id) {


    case CAN_ID_HOMING_CMD: {
      if (msg.len == 1 && msg.buf[0] == 1) {
        homingRequested = true;
      }
      break;
    }

    case CAN_ID_SET_SERVO_POS: {
      if (msg.len == 8) {
        servoTargetDeg[0] = unpack_i32_le(&msg.buf[0]);
        servoTargetDeg[1] = unpack_i32_le(&msg.buf[4]);
        servoTargetsUpdated = true;
      }
      break;
    }

    case CAN_ID_SET_POS_01: {
      if (!ignoreCanPosTargets && msg.len == 8) {
        posTarget[0] = unpack_i32_le(&msg.buf[0]);
        posTarget[1] = unpack_i32_le(&msg.buf[4]);
        posTargetsUpdated = true;
        posTargetsUpdatedAgeMs = 0;
      }
      break;
    }

    case CAN_ID_SET_POS_23: {
      if (!ignoreCanPosTargets && msg.len == 8) {
        posTarget[2] = unpack_i32_le(&msg.buf[0]);
        posTarget[3] = unpack_i32_le(&msg.buf[4]);
        posTargetsUpdated = true;
        posTargetsUpdatedAgeMs = 0;
      }
      break;
    }

    case CAN_ID_SET_POS_45: {
      if (!ignoreCanPosTargets && msg.len == 8) {
        posTarget[4] = unpack_i32_le(&msg.buf[0]);
        posTarget[5] = unpack_i32_le(&msg.buf[4]);
        posTargetsUpdated = true;
        posTargetsUpdatedAgeMs = 0;
      }
      break;
    }

    default:
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

void servoISR0() {
  if (!servoPulseHigh0) {
    digitalWriteFast(SERVO0, LOW);
    servoPulseHigh0 = true;

    uint32_t t = servoPulseWidthUs0;
    if (t < 1) t = 1;
    servoTimer0.update(t);
  } else {
    digitalWriteFast(SERVO0, HIGH);
    servoPulseHigh0 = false;

    uint32_t t = SERVO_FRAME_US - servoPulseWidthUs0;
    if (t < 1) t = 1;
    servoTimer0.update(t);
  }
}

void servoISR1() {
  if (!servoPulseHigh1) {
    digitalWriteFast(SERVO1, LOW);
    servoPulseHigh1 = true;

    uint32_t t = servoPulseWidthUs1;
    if (t < 1) t = 1;
    servoTimer1.update(t);
  } else {
    digitalWriteFast(SERVO1, HIGH);
    servoPulseHigh1 = false;

    uint32_t t = SERVO_FRAME_US - servoPulseWidthUs1;
    if (t < 1) t = 1;
    servoTimer1.update(t);
  }
}

void setServoPulseUs(uint8_t servoIndex, uint32_t us) {
  if (us < SERVO_MIN_PULSE_US) us = SERVO_MIN_PULSE_US;
  if (us > SERVO_MAX_PULSE_US) us = SERVO_MAX_PULSE_US;

  noInterrupts();
  if (servoIndex == 0) {
    servoPulseWidthUs0 = us;
  } else if (servoIndex == 1) {
    servoPulseWidthUs1 = us;
  }
  interrupts();
}

void setServoAngle(uint8_t servoIndex, float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > SERVO_RANGE_DEG) deg = SERVO_RANGE_DEG;

  const float span = (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
  const uint32_t us = (uint32_t)(SERVO_MIN_PULSE_US + (deg / SERVO_RANGE_DEG) * span);
  setServoPulseUs(servoIndex, us);
}

void applyServoTargets() {
  int32_t target0;
  int32_t target1;

  noInterrupts();
  target0 = servoTargetDeg[0];
  target1 = servoTargetDeg[1];
  interrupts();

  setServoAngle(0, (float)target0);
  setServoAngle(1, (float)target1);
}

static inline int32_t unpack_i32_le(const uint8_t *b) {
  return (int32_t)b[0]
       | ((int32_t)b[1] << 8)
       | ((int32_t)b[2] << 16)
       | ((int32_t)b[3] << 24);
}

static inline void pack_i32_le(uint8_t *b, int32_t v) {
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  b[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void enterPosModeHold(int index) {
  // Hold current position so we don't jump
  const int32_t pos = (int32_t)encoders[index]->read();
  noInterrupts();
  posTarget[index] = pos;
  interrupts();

  integ[index] = 0.0f;
  prevErr[index] = 0;

  ControlMode[index] = MODE_POS;
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

static inline float readMotorCurrentA(int index) {
  constexpr float VADC = 3.3f;
  constexpr float ADC_MAX = 4095.0f;
  constexpr float RIPROPI_OHMS = 5100.0f;
  constexpr float AIPROPI_A_PER_A = 450e-6f; // 450 uA/A

  const int raw = analogRead(motorSEN[index]);
  const float v = raw * (VADC / ADC_MAX);
  return v / (AIPROPI_A_PER_A * RIPROPI_OHMS);
}

// --- Motor output modes using your PWM_A/PWM_B arrays ---
// dir: +1 forward, -1 reverse
static inline void setDriveCoast(int index, int dir, uint8_t duty) {
  duty = constrain(duty, 0, 255);
  if (dir > 0) { PWM_A[index] = duty; PWM_B[index] = 0; }
  else         { PWM_A[index] = 0;    PWM_B[index] = duty; }
  controlMotor(index);
}

// Drive–Brake PWM: OFF portion becomes (1,1) brake.
// driveDuty sets effective "drive fraction" similar to coast PWM.
static inline void setDriveBrake(int index, int dir, uint8_t driveDuty) {
  driveDuty = constrain(driveDuty, 0, 255);
  const uint8_t brakeDuty = 255 - driveDuty;  // how much of the cycle is BRAKE

  if (dir > 0) {  // forward: (1,0)=drive, (1,1)=brake
    PWM_A[index] = 255;
    PWM_B[index] = brakeDuty;
  } else {        // reverse: (0,1)=drive, (1,1)=brake
    PWM_B[index] = 255;
    PWM_A[index] = brakeDuty;
  }
  controlMotor(index);
}

#if 0
static bool homingStallDetectedTimed(int index,
                                    float stallA,
                                    uint32_t debounceMs,
                                    uint32_t sampleMs,
                                    uint32_t maxMs,
                                    bool &faulted,
                                    uint32_t blankMs) {
  // NOTE: nFAULT is intentionally ignored for homing.
  // We rely on current-sense stall detection + timeouts instead.
  // blankMs: ignore current sensing for the first blankMs after starting this detection window.
  faulted = false;
  uint32_t aboveStart = 0;
  const uint32_t t0 = millis();

  while (millis() - t0 < maxMs) {
    const uint32_t elapsed = millis() - t0;

    // Startup blanking window (useful for drive–brake where current can spike immediately)
    if (elapsed < blankMs) {
      aboveStart = 0;
      delay(sampleMs);
      continue;
    }

    const float iA = readMotorCurrentA(index);

    if (iA >= stallA) {
      if (aboveStart == 0) aboveStart = millis();
      if (millis() - aboveStart >= debounceMs) return true;
    } else {
      aboveStart = 0;
    }

    delay(sampleMs);
  }

  return false;
}

static void homingBackoffFrom(int index,
                              int approachDir,
                              int counts,
                              uint8_t duty,
                              uint32_t timeoutMs) {
  const long start = encoders[index]->read();
  const int away = -approachDir;

  setDriveCoast(index, away, duty);
  const uint32_t t0 = millis();

  while (millis() - t0 < timeoutMs) {
    const long now = encoders[index]->read();
    const long d = now - start;
    if ((away > 0 && d >= counts) || (away < 0 && d <= -counts)) break;
  }

  stopMotor(index);
}
#endif

static inline long median3(long a, long b, long c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

static long trimmedMean(const long *samples, int n) {
  if (!samples || n <= 0) return 0;
  if (n == 1) return samples[0];

  long mn = samples[0], mx = samples[0];
  long sum = 0;
  for (int i = 0; i < n; i++) {
    mn = min(mn, samples[i]);
    mx = max(mx, samples[i]);
    sum += samples[i];
  }

  if (n >= 3) {
    sum -= (mn + mx);
    return sum / (n - 2);
  }
  return sum / n;
}

static bool homingMotionReached(int index, int dir, int minDeltaCounts) {
  const long now = encoders[index]->read();
  const long delta = now - homingState[index].phaseStartPos;
  return (dir > 0) ? (delta >= minDeltaCounts) : (delta <= -minDeltaCounts);
}

static int homingTouchRepeatsClamped(int index) {
  return constrain((int)homingTouchRepeats[index], 1, HOMING_MAX_TOUCH_SAMPLES);
}

static bool homingStallDetectedNonBlocking(int index,
                                           float stallA,
                                           uint32_t debounceMs,
                                           uint32_t sampleMs,
                                           uint32_t maxMs,
                                           uint32_t blankMs) {
  HomingState &state = homingState[index];
  const uint32_t now = millis();
  const uint32_t elapsed = now - state.phaseStartMs;

  if (elapsed >= maxMs) {
    return false;
  }

  if (elapsed < blankMs) {
    state.aboveStartMs = 0;
    return false;
  }

  if ((now - state.lastSampleMs) < sampleMs) {
    return false;
  }

  state.lastSampleMs = now;

  const float currentA = readMotorCurrentA(index);
  if (currentA >= stallA) {
    if (state.aboveStartMs == 0) {
      state.aboveStartMs = now;
    }
    if ((now - state.aboveStartMs) >= debounceMs) {
      return true;
    }
  } else {
    state.aboveStartMs = 0;
  }

  return false;
}

static void refreshHomingVref() {
  int requestedDuty = NORMAL_VREF_DUTY;
  bool anyActive = false;

  for (int i = 0; i < 6; i++) {
    if (homingState[i].active) {
      requestedDuty = anyActive ? max(requestedDuty, homingVoltage[i]) : homingVoltage[i];
      anyActive = true;
    }
  }

  setVrefDuty(requestedDuty);
}

static void enterHomingPhase(int index, HomingPhase phase) {
  HomingState &state = homingState[index];
  state.phase = phase;
  state.phaseStartMs = millis();
  state.lastSampleMs = 0;
  state.aboveStartMs = 0;
  state.phaseStartPos = encoders[index]->read();
}

static void finishHomingMotor(int index, bool success, const char *message) {
  HomingState &state = homingState[index];

  stopMotor(index);
  state.active = false;
  state.phase = HOMING_IDLE;
  ControlMode[index] = MODE_POS;

  if (!success) {
    enterPosModeHold(index);
    Serial.print("Homing aborted for motor ");
    Serial.print(index);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.print("Homing complete for motor ");
    Serial.print(index);
    Serial.print(". low=");
    Serial.print(lowROM[index]);
    Serial.print(" high=");
    Serial.print(highROM[index]);
    Serial.print(" mid=");
    Serial.println((lowROM[index] + highROM[index]) / 2);
  }

  ignoreCanPosTargets = true;
  ignoreCanPosTargetsAgeMs = 0;
  refreshHomingVref();
  sendRangeOfMotion();
}

void startHomingMotor(int index) {
  HomingState &state = homingState[index];

  state = HomingState{};
  state.active = true;
  state.touchRepeatTarget = (uint8_t)homingTouchRepeatsClamped(index);
  ignoreCanPosTargets = true;
  ignoreCanPosTargetsAgeMs = 0;

  ControlMode[index] = MODE_PWM;
  stopMotor(index);
  refreshHomingVref();
  enterHomingPhase(index, HOMING_VREF_SETTLE);

  Serial.print("Starting homing for motor ");
  Serial.println(index);
}

void beginHomingSequence() {
  bool startedAny = false;

  for (int i = 0; i < HOMING_SEQUENCE_LEN; i++) {
    const int motor = HOMING_SEQUENCE[i];
    if (motor < 0 || motor >= 6) {
      continue;
    }

    if (!homingState[motor].active) {
      startHomingMotor(motor);
      startedAny = true;
    }
  }

  homingActive = startedAny || homingActive;
}

void updateHomingMotor(int index) {
  HomingState &state = homingState[index];
  if (!state.active) return;

  const uint32_t nowMs = millis();
  const uint32_t phaseElapsedMs = nowMs - state.phaseStartMs;

  const float    STALL_BLUNT_A     = homingStallA_blunt[index];
  const float    STALL_PRECISE_A   = homingStallA_precise[index];
  const uint32_t BRAKE_BLANK_MS    = homingBrakeBlankMs[index];
  const uint32_t STALL_DEBOUNCE_MS = homingStallDebounceMs[index];
  const uint32_t SAMPLE_MS         = homingSampleMs[index];

  const uint8_t  UNSTICK_DUTY       = homingUnstickDuty[index];
  const uint32_t UNSTICK_MS         = homingUnstickMs[index];
  const int      UNSTICK_MIN_COUNTS = homingUnstickMinCounts[index];

  const uint8_t  BLUNT_DUTY         = homingBluntDuty[index];
  const int      BLUNT_BACKOFF      = homingBluntBackoffCounts[index];
  const uint32_t BLUNT_BACKOFF_MS   = homingBluntBackoffTimeoutMs[index];
  const uint8_t  BLUNT_BACKOFF_DUTY = 255;

  const uint8_t  PRECISE_DUTY       = homingPreciseDuty[index];
  const uint32_t PRECISE_TIMEOUT_MS = homingPreciseTimeoutMs[index];
  const int      TOUCH_BACKOFF      = homingTouchBackoffCounts[index];
  const uint32_t TOUCH_BACKOFF_MS   = homingTouchBackoffTimeoutMs[index];
  const uint8_t  TOUCH_BACKOFF_DUTY = homingTouchBackoffDuty[index];

  const uint32_t MID_TIMEOUT_MS     = homingMidTimeoutMs[index];
  const int      MID_TOL_COUNTS     = homingMidTolCounts[index];

  constexpr int DIR_LOW  = -1;
  constexpr int DIR_HIGH = +1;

  switch (state.phase) {
    case HOMING_VREF_SETTLE:
      if (phaseElapsedMs >= VREF_SETTLE_MS) {
        Serial.print("M"); Serial.print(index); Serial.println(" Step 1: Unstick");
        setDriveCoast(index, +1, UNSTICK_DUTY);
        enterHomingPhase(index, HOMING_STEP1_UNSTICK_FWD);
      }
      break;

    case HOMING_STEP1_UNSTICK_FWD:
      if (homingMotionReached(index, +1, UNSTICK_MIN_COUNTS)) {
        stopMotor(index);
        Serial.print("M"); Serial.print(index); Serial.println(" Step 2: Blunt Low");
        setDriveBrake(index, DIR_LOW, BLUNT_DUTY);
        enterHomingPhase(index, HOMING_STEP2_BLUNT_LOW);
      } else if (phaseElapsedMs >= UNSTICK_MS) {
        stopMotor(index);
        setDriveCoast(index, -1, UNSTICK_DUTY);
        enterHomingPhase(index, HOMING_STEP1_UNSTICK_REV);
      }
      break;

    case HOMING_STEP1_UNSTICK_REV:
      if (homingMotionReached(index, -1, UNSTICK_MIN_COUNTS)) {
        stopMotor(index);
        Serial.print("M"); Serial.print(index); Serial.println(" Step 2: Blunt Low");
        setDriveBrake(index, DIR_LOW, BLUNT_DUTY);
        enterHomingPhase(index, HOMING_STEP2_BLUNT_LOW);
      } else if (phaseElapsedMs >= UNSTICK_MS) {
        finishHomingMotor(index, false, "could not unstick");
      }
      break;

    case HOMING_STEP2_BLUNT_LOW:
      if (homingStallDetectedNonBlocking(index, STALL_BLUNT_A, STALL_DEBOUNCE_MS, SAMPLE_MS,
                                         homingBluntTimeoutLowMs[index], BRAKE_BLANK_MS)) {
        stopMotor(index);
        Serial.print("M"); Serial.print(index);
        Serial.print(" blunt low hit @ ");
        Serial.println(encoders[index]->read());
        setDriveCoast(index, -DIR_LOW, BLUNT_BACKOFF_DUTY);
        enterHomingPhase(index, HOMING_STEP2_BACKOFF_LOW);
      } else if (phaseElapsedMs >= homingBluntTimeoutLowMs[index]) {
        finishHomingMotor(index, false, "blunt low timeout");
      }
      break;

    case HOMING_STEP2_BACKOFF_LOW:
      if (homingMotionReached(index, -DIR_LOW, BLUNT_BACKOFF) || phaseElapsedMs >= BLUNT_BACKOFF_MS) {
        stopMotor(index);
        Serial.print("M"); Serial.print(index); Serial.println(" Step 3: Precise Low");
        setDriveBrake(index, DIR_LOW, PRECISE_DUTY);
        enterHomingPhase(index, HOMING_STEP3_PRECISE_LOW_DRIVE);
      }
      break;

    case HOMING_STEP3_PRECISE_LOW_DRIVE:
      if (homingStallDetectedNonBlocking(index, STALL_PRECISE_A, STALL_DEBOUNCE_MS, SAMPLE_MS,
                                         PRECISE_TIMEOUT_MS, BRAKE_BLANK_MS)) {
        stopMotor(index);
        state.lowSamples[state.touchRepeatIndex] = encoders[index]->read();
        Serial.print("M"); Serial.print(index);
        Serial.print(" precise low sample ");
        Serial.print(state.touchRepeatIndex + 1);
        Serial.print("/");
        Serial.print(state.touchRepeatTarget);
        Serial.print(" @ ");
        Serial.println(state.lowSamples[state.touchRepeatIndex]);

        if ((state.touchRepeatIndex + 1) >= state.touchRepeatTarget) {
          state.lowEnd = (state.touchRepeatTarget == 3)
            ? median3(state.lowSamples[0], state.lowSamples[1], state.lowSamples[2])
            : ((state.touchRepeatTarget > 1) ? trimmedMean(state.lowSamples, state.touchRepeatTarget)
                                             : state.lowSamples[0]);
          lowROM[index] = state.lowEnd;
          Serial.print("M"); Serial.print(index);
          Serial.print(" averaged low endpoint = ");
          Serial.println(state.lowEnd);
          state.touchRepeatIndex = state.touchRepeatTarget;
          Serial.print("M"); Serial.print(index);
          Serial.print(" final low-side exit backoff: duty=");
          Serial.print(TOUCH_BACKOFF_DUTY);
          Serial.print(" counts=");
          Serial.println(TOUCH_BACKOFF);
          setDriveCoast(index, -DIR_LOW, TOUCH_BACKOFF_DUTY);
          enterHomingPhase(index, HOMING_STEP3_PRECISE_LOW_BACKOFF);
        } else {
          state.touchRepeatIndex++;
          Serial.print("M"); Serial.print(index);
          Serial.print(" hard backoff low: duty=");
          Serial.print(TOUCH_BACKOFF_DUTY);
          Serial.print(" counts=");
          Serial.println(TOUCH_BACKOFF);
          setDriveCoast(index, -DIR_LOW, TOUCH_BACKOFF_DUTY);
          enterHomingPhase(index, HOMING_STEP3_PRECISE_LOW_BACKOFF);
        }
      } else if (phaseElapsedMs >= PRECISE_TIMEOUT_MS) {
        finishHomingMotor(index, false, "precise low timeout");
      }
      break;

    case HOMING_STEP3_PRECISE_LOW_BACKOFF:
      if (homingMotionReached(index, -DIR_LOW, TOUCH_BACKOFF) || phaseElapsedMs >= TOUCH_BACKOFF_MS) {
        stopMotor(index);
        if (state.touchRepeatIndex >= state.touchRepeatTarget) {
          Serial.print("M"); Serial.print(index); Serial.println(" Step 4: Blunt High");
          setDriveBrake(index, DIR_HIGH, BLUNT_DUTY);
          enterHomingPhase(index, HOMING_STEP4_BLUNT_HIGH);
        } else {
          enterHomingPhase(index, HOMING_STEP3_PRECISE_LOW_SETTLE);
        }
      }
      break;

    case HOMING_STEP3_PRECISE_LOW_SETTLE:
      if (phaseElapsedMs >= HOMING_TOUCH_SETTLE_MS) {
        setDriveBrake(index, DIR_LOW, PRECISE_DUTY);
        enterHomingPhase(index, HOMING_STEP3_PRECISE_LOW_DRIVE);
      }
      break;

    case HOMING_STEP4_BLUNT_HIGH:
      if (homingStallDetectedNonBlocking(index, STALL_BLUNT_A, STALL_DEBOUNCE_MS, SAMPLE_MS,
                                         homingBluntTimeoutHighMs[index], BRAKE_BLANK_MS)) {
        stopMotor(index);
        Serial.print("M"); Serial.print(index);
        Serial.print(" blunt high hit @ ");
        Serial.println(encoders[index]->read());
        setDriveCoast(index, -DIR_HIGH, BLUNT_BACKOFF_DUTY);
        enterHomingPhase(index, HOMING_STEP4_BACKOFF_HIGH);
      } else if (phaseElapsedMs >= homingBluntTimeoutHighMs[index]) {
        finishHomingMotor(index, false, "blunt high timeout");
      }
      break;

    case HOMING_STEP4_BACKOFF_HIGH:
      if (homingMotionReached(index, -DIR_HIGH, BLUNT_BACKOFF) || phaseElapsedMs >= BLUNT_BACKOFF_MS) {
        stopMotor(index);
        Serial.print("M"); Serial.print(index); Serial.println(" Step 5: Precise High");
        state.touchRepeatIndex = 0;
        setDriveBrake(index, DIR_HIGH, PRECISE_DUTY);
        enterHomingPhase(index, HOMING_STEP5_PRECISE_HIGH_DRIVE);
      }
      break;

    case HOMING_STEP5_PRECISE_HIGH_DRIVE:
      if (homingStallDetectedNonBlocking(index, STALL_PRECISE_A, STALL_DEBOUNCE_MS, SAMPLE_MS,
                                         PRECISE_TIMEOUT_MS, BRAKE_BLANK_MS)) {
        stopMotor(index);
        state.highSamples[state.touchRepeatIndex] = encoders[index]->read();
        Serial.print("M"); Serial.print(index);
        Serial.print(" precise high sample ");
        Serial.print(state.touchRepeatIndex + 1);
        Serial.print("/");
        Serial.print(state.touchRepeatTarget);
        Serial.print(" @ ");
        Serial.println(state.highSamples[state.touchRepeatIndex]);

        if ((state.touchRepeatIndex + 1) >= state.touchRepeatTarget) {
          state.highEnd = (state.touchRepeatTarget == 3)
            ? median3(state.highSamples[0], state.highSamples[1], state.highSamples[2])
            : ((state.touchRepeatTarget > 1) ? trimmedMean(state.highSamples, state.touchRepeatTarget)
                                             : state.highSamples[0]);
          highROM[index] = state.highEnd;
          Serial.print("M"); Serial.print(index);
          Serial.print(" averaged high endpoint = ");
          Serial.println(state.highEnd);

          state.touchRepeatIndex = state.touchRepeatTarget;
          Serial.print("M"); Serial.print(index);
          Serial.print(" final high-side exit backoff: duty=");
          Serial.print(TOUCH_BACKOFF_DUTY);
          Serial.print(" counts=");
          Serial.println(TOUCH_BACKOFF);
          setDriveCoast(index, -DIR_HIGH, TOUCH_BACKOFF_DUTY);
          enterHomingPhase(index, HOMING_STEP5_PRECISE_HIGH_BACKOFF);
        } else {
          state.touchRepeatIndex++;
          Serial.print("M"); Serial.print(index);
          Serial.print(" hard backoff high: duty=");
          Serial.print(TOUCH_BACKOFF_DUTY);
          Serial.print(" counts=");
          Serial.println(TOUCH_BACKOFF);
          setDriveCoast(index, -DIR_HIGH, TOUCH_BACKOFF_DUTY);
          enterHomingPhase(index, HOMING_STEP5_PRECISE_HIGH_BACKOFF);
        }
      } else if (phaseElapsedMs >= PRECISE_TIMEOUT_MS) {
        finishHomingMotor(index, false, "precise high timeout");
      }
      break;

    case HOMING_STEP5_PRECISE_HIGH_BACKOFF:
      if (homingMotionReached(index, -DIR_HIGH, TOUCH_BACKOFF) || phaseElapsedMs >= TOUCH_BACKOFF_MS) {
        stopMotor(index);
        if (state.touchRepeatIndex >= state.touchRepeatTarget) {
          const long mid = (state.lowEnd + state.highEnd) / 2;
          Serial.print("M"); Serial.print(index);
          Serial.print(" midpoint target = ");
          Serial.println(mid);
          Serial.print("M"); Serial.print(index); Serial.print(" Step 6: Move to midpoint = ");
          Serial.println(mid);

          integ[index] = 0.0f;
          prevErr[index] = 0;
          noInterrupts();
          posTarget[index] = mid;
          interrupts();
          ControlMode[index] = MODE_POS;
          enterHomingPhase(index, HOMING_STEP6_MOVE_MID);
        } else {
          enterHomingPhase(index, HOMING_STEP5_PRECISE_HIGH_SETTLE);
        }
      }
      break;

    case HOMING_STEP5_PRECISE_HIGH_SETTLE:
      if (phaseElapsedMs >= HOMING_TOUCH_SETTLE_MS) {
        setDriveBrake(index, DIR_HIGH, PRECISE_DUTY);
        enterHomingPhase(index, HOMING_STEP5_PRECISE_HIGH_DRIVE);
      }
      break;

    case HOMING_STEP6_MOVE_MID: {
      const long mid = (state.lowEnd + state.highEnd) / 2;
      const int32_t err = mid - (int32_t)encoders[index]->read();

      if (abs(err) <= MID_TOL_COUNTS) {
        finishHomingMotor(index, true, "done");
      } else if (phaseElapsedMs >= MID_TIMEOUT_MS) {
        finishHomingMotor(index, false, "midpoint timeout");
      }
      break;
    }

    case HOMING_IDLE:
    default:
      break;
  }
}

void updateHomingSequence() {
  bool anyActive = false;

  for (int i = 0; i < 6; i++) {
    if (!homingState[i].active) {
      continue;
    }

    anyActive = true;
    updateHomingMotor(i);
  }

  if (homingActive && !anyActive) {
    homingActive = false;
    Serial.println("Homing sequence complete");
  } else if (anyActive) {
    homingActive = true;
  }
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

void printPositionControllerDiagnostics() {
  // Snapshot targets atomically (even if you also snapshot elsewhere, this is self-contained)
  int32_t t[6];
  noInterrupts();
  for (int i = 0; i < 6; i++) t[i] = posTarget[i];
  interrupts();

  Serial.print("POS CTRL (age ");
  Serial.print((uint32_t)posTargetsUpdatedAgeMs);
  Serial.println(" ms)");

  for (int i = 0; i < 6; i++) {
    const int32_t pos = (int32_t)encoders[i]->read();
    const int32_t err = t[i] - pos;

    Serial.print("M"); Serial.print(i);
    Serial.print(": tgt="); Serial.print(t[i]);
    Serial.print(" pos="); Serial.print(pos);
    Serial.print(" err="); Serial.print(err);
    Serial.println();
  }
}

void sendEncoderPositions() {
  for (int i = 0; i < 6; i++) {
    CAN_message_t msg;
    msg.id  = CAN_ID_ENCODER_BASE + i;
    msg.len = 4;

    int32_t pos = (int32_t)encoders[i]->read();
    pack_i32_le(msg.buf, pos);  

    Can3.write(msg);
  }
}

void sendRangeOfMotion() {
  for (int i = 0; i < 6; i++) {
    CAN_message_t msg;
    msg.id = CAN_ID_ROM_BASE + i;
    msg.len = 8;

    pack_i32_le(&msg.buf[0], (int32_t)lowROM[i]);
    pack_i32_le(&msg.buf[4], (int32_t)highROM[i]);

    Can3.write(msg);
  }
}

static inline uint8_t applyMinDuty(uint8_t duty, uint8_t minDuty) {
  if (duty == 0) return 0;
  return (duty < minDuty) ? minDuty : duty;
}

// Call this periodically (e.g., every 5ms) when MODE_POS
void updatePositionController(int index, float dt_s) {
  // Snapshot target atomically (CAN ISR updates it)
  int32_t tgt;
  noInterrupts();
  tgt = posTarget[index];
  interrupts();

  //Crude ROM clamping
  const int32_t lo = (int32_t)lowROM[index];
  const int32_t hi = (int32_t)highROM[index];

  if (tgt < lo) tgt = lo;
  if (tgt > hi) tgt = hi;

  const int32_t pos = (int32_t)encoders[index]->read();
  const int32_t err = tgt - pos;

  // Deadband = behave like a servo: "good enough" -> stop
  if (abs(err) <= posDeadbandCounts[index]) {
    // Stop motor in coast
    PWM_A[index] = 0;
    PWM_B[index] = 0;
    controlMotor(index);

    // Optional: keep integrator from winding (PI-ready)
    prevErr[index] = err;
    return;
  }
  //------------------
  // --- PID form  ---
  //------------------
  // P
  float u = Kp[index] * (float)err;

  // I (not used yet, but ready)
   integ[index] += (float)err * dt_s;
   u += Ki[index] * integ[index];

  // D (not used yet, but ready)
  // const float derr = ((float)err - (float)prevErr[index]) / dt_s;
  // u += Kd[index] * derr;

  prevErr[index] = err;

  // Convert control effort -> signed duty command
  // u is "duty-ish". Clamp to [-posMaxDuty, +posMaxDuty]
  const float umax = (float)posMaxDuty[index];
  if (u >  umax) u =  umax;
  if (u < -umax) u = -umax;

  const int dir = (u >= 0.0f) ? +1 : -1;
  uint8_t duty = (uint8_t)abs((int)u);

  // Apply minimum duty to overcome stiction (servo-like)
  duty = applyMinDuty(duty, posMinDuty[index]);

  // Drive using your existing coast mode
  setDriveCoast(index, dir, duty);
}

#if 0
// Legacy blocking homing path kept only as reference while the non-blocking
// state machine is being tuned. It is no longer called by loop() or CAN.
[[maybe_unused]] static void homeMotor(int index) {
  Serial.print("Homing motor ");
  Serial.println(index);

  // Pull per-motor parameters
  const float    STALL_BLUNT_A     = homingStallA_blunt[index];
  const float    STALL_PRECISE_A   = homingStallA_precise[index];
  const uint32_t BRAKE_BLANK_MS    = homingBrakeBlankMs[index];
  const uint32_t STALL_DEBOUNCE_MS = homingStallDebounceMs[index];
  const uint32_t SAMPLE_MS         = homingSampleMs[index];

  const uint8_t  UNSTICK_DUTY       = homingUnstickDuty[index];
  const uint32_t UNSTICK_MS         = homingUnstickMs[index];
  const int      UNSTICK_MIN_COUNTS = homingUnstickMinCounts[index];

  const uint8_t  BLUNT_DUTY         = homingBluntDuty[index];
  const int      BLUNT_BACKOFF      = homingBluntBackoffCounts[index];
  const uint32_t BLUNT_BACKOFF_MS   = homingBluntBackoffTimeoutMs[index];

  const uint8_t  PRECISE_DUTY       = homingPreciseDuty[index];
  const uint32_t PRECISE_TIMEOUT_MS = homingPreciseTimeoutMs[index];
  int            TOUCH_REPEATS      = (int)homingTouchRepeats[index];
  const int      TOUCH_BACKOFF      = homingTouchBackoffCounts[index];
  const uint32_t TOUCH_BACKOFF_MS   = homingTouchBackoffTimeoutMs[index];

  const uint32_t MID_TIMEOUT_MS     = homingMidTimeoutMs[index];
  const int      MID_TOL_COUNTS     = homingMidTolCounts[index];

  // Clamp repeats to something sane for stack arrays
  constexpr int MAX_TOUCH = 5;
  TOUCH_REPEATS = constrain(TOUCH_REPEATS, 1, MAX_TOUCH);

  // Define which direction is "LOW" for this motor.
  // Convention used here: LOW = -1, HIGH = +1
  const int DIR_LOW  = -1;
  const int DIR_HIGH = +1;

  // NOTE: nFAULT is intentionally ignored during homing (no pre-clear / no abort-on-fault)

  // Set homing current limit (VREF) and settle
  setVrefDuty(homingVoltage[index]);
  delay(VREF_SETTLE_MS);
  ControlMode[index] = MODE_PWM; //Switching over to open loop PWM control for homing

  // ------------------------------
  // Step 1: Unstick
  // ------------------------------

  Serial.println("Step 1: Unstick");
  setDriveCoast(index, +1, UNSTICK_DUTY);
  bool moved = waitForMotionCounts(index, +1, UNSTICK_MIN_COUNTS, UNSTICK_MS);
  stopMotor(index);

  if (!moved) {
    setDriveCoast(index, -1, UNSTICK_DUTY);
    moved = waitForMotionCounts(index, -1, UNSTICK_MIN_COUNTS, UNSTICK_MS);
    stopMotor(index);
  }

  if (!moved) {
    Serial.println("Abort: could not unstick (no encoder motion). ");
    setVrefDuty(NORMAL_VREF_DUTY);
    enterPosModeHold(index);  
    return;
  }

  // ------------------------------
  // Step 2: Blunt Low End (drive–coast)
  // ------------------------------
  Serial.println("Step 2: Blunt Low (drive–coast)");
  setDriveCoast(index, DIR_LOW, BLUNT_DUTY);

  bool faulted = false;
  bool hitLow = homingStallDetectedTimed(index, STALL_BLUNT_A, STALL_DEBOUNCE_MS, SAMPLE_MS, homingBluntTimeoutLowMs[index], faulted, 0);
  stopMotor(index);

  // NOTE: nFAULT ignored during homing; we do not abort on faulted here.
  if (!hitLow) {
    Serial.println("Abort: blunt low timeout (no stall detected). ");
    setVrefDuty(NORMAL_VREF_DUTY);
    enterPosModeHold(index);
    return;
  }

  // Back off to create a known neighborhood
  homingBackoffFrom(index, DIR_LOW, BLUNT_BACKOFF, BLUNT_DUTY, BLUNT_BACKOFF_MS);

  // ------------------------------
  // Step 3: Precise Low End (drive–brake), optional multi-touch
  // ------------------------------
  Serial.println("Step 3: Precise Low (drive–brake)");
  long lowSamples[MAX_TOUCH] = {0};

  for (int k = 0; k < TOUCH_REPEATS; k++) {
    setDriveBrake(index, DIR_LOW, PRECISE_DUTY);
    bool stalled = homingStallDetectedTimed(index, STALL_PRECISE_A, STALL_DEBOUNCE_MS, SAMPLE_MS, PRECISE_TIMEOUT_MS, faulted, BRAKE_BLANK_MS);
    stopMotor(index);

    // NOTE: nFAULT ignored during homing; we do not abort on faulted here.
    if (!stalled) {
      Serial.println("Abort: precise low timeout (no stall detected). ");
      setVrefDuty(NORMAL_VREF_DUTY);
      enterPosModeHold(index);
      return;
    }

    lowSamples[k] = encoders[index]->read();

    Serial.print("Sample ");
    Serial.print(k);
    Serial.print(":");
    Serial.println(lowSamples[k]);

    homingBackoffFrom(index, DIR_LOW, TOUCH_BACKOFF, (uint8_t)max(15, (int)PRECISE_DUTY), TOUCH_BACKOFF_MS);
    delay(25);
  }

  long lowEnd = lowSamples[0];
  if (TOUCH_REPEATS == 3) {
    lowEnd = median3(lowSamples[0], lowSamples[1], lowSamples[2]);
  } else if (TOUCH_REPEATS > 1) {
    lowEnd = trimmedMean(lowSamples, TOUCH_REPEATS);
  }

  lowROM[index] = lowEnd;

  // ------------------------------
  // Step 4: Blunt High End (drive–coast)
  // ------------------------------
  Serial.println("Step 4: Blunt High (drive–coast)");
  setDriveCoast(index, DIR_HIGH, BLUNT_DUTY);

  bool hitHigh = homingStallDetectedTimed(index, STALL_BLUNT_A, STALL_DEBOUNCE_MS, SAMPLE_MS, homingBluntTimeoutHighMs[index], faulted, 0);
  stopMotor(index);

  // NOTE: nFAULT ignored during homing; we do not abort on faulted here.
  if (!hitHigh) {
    Serial.println("Abort: blunt high timeout (no stall detected). ");
    setVrefDuty(NORMAL_VREF_DUTY);
    enterPosModeHold(index);
    return;
  }

  homingBackoffFrom(index, DIR_HIGH, BLUNT_BACKOFF, (uint8_t)max(20, (int)BLUNT_DUTY - 30), BLUNT_BACKOFF_MS);

  // ------------------------------
  // Step 5: Precise High End (drive–brake), optional multi-touch
  // ------------------------------
  Serial.println("Step 5: Precise High (drive–brake)");
  long highSamples[MAX_TOUCH] = {0};

  for (int k = 0; k < TOUCH_REPEATS; k++) {
    setDriveBrake(index, DIR_HIGH, PRECISE_DUTY);
    bool stalled = homingStallDetectedTimed(index, STALL_PRECISE_A, STALL_DEBOUNCE_MS, SAMPLE_MS, PRECISE_TIMEOUT_MS, faulted, BRAKE_BLANK_MS);
    stopMotor(index);

    // NOTE: nFAULT ignored during homing; we do not abort on faulted here.
    if (!stalled) {
      Serial.println("Abort: precise high timeout (no stall detected). ");
      setVrefDuty(NORMAL_VREF_DUTY);
      enterPosModeHold(index);
      return;
    }

    highSamples[k] = encoders[index]->read();

    Serial.print("Sample ");
    Serial.print(k);
    Serial.print(":");
    Serial.println(highSamples[k]);

    homingBackoffFrom(index, DIR_HIGH, TOUCH_BACKOFF, (uint8_t)max(15, (int)PRECISE_DUTY), TOUCH_BACKOFF_MS);
    delay(25);
  }

  long highEnd = highSamples[0];
  if (TOUCH_REPEATS == 3) {
    highEnd = median3(highSamples[0], highSamples[1], highSamples[2]);
  } else if (TOUCH_REPEATS > 1) {
    highEnd = trimmedMean(highSamples, TOUCH_REPEATS);
  }

  highROM[index] = highEnd;

  // ------------------------------
  // Step 6: Move to midpoint
  // ------------------------------
  const long mid = (lowEnd + highEnd) / 2;
  Serial.print("Step 6: Move to midpoint = ");
  Serial.println(mid);
  ControlMode[index] = MODE_POS;  // Ensure we leave homing in position mode

  posTarget[index] = mid;

  // Run the POS controller until centered or timeout
  bool centered = false;
  const uint32_t t0 = millis();
  uint32_t lastUs = micros();

  while (millis() - t0 < MID_TIMEOUT_MS) {
    const uint32_t nowUs = micros();
    const float dt_s = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    // Run your servo-like loop
    updatePositionController(index, dt_s);

    // Check if we’re “close enough”
    const int32_t pos = (int32_t)encoders[index]->read();
    const int32_t err = mid - pos;
    if (abs(err) <= MID_TOL_COUNTS) {
      centered = true;
      break;
    }

    // Keep loop from hogging CPU / spamming encoder reads
    delayMicroseconds(2000); // ~500 Hz-ish
  }

  if (!centered) {
    Serial.println("Abort: midpoint move timeout (POS mode).");
    setVrefDuty(NORMAL_VREF_DUTY);
    enterPosModeHold(index);
    return;
  }

  // Restore normal VREF
  setVrefDuty(NORMAL_VREF_DUTY);

  Serial.print("Homing complete. low=");
  Serial.print(lowROM[index]);
  Serial.print(" high=");
  Serial.print(highROM[index]);
  Serial.print(" mid=");
  Serial.println(mid);
}
#endif

