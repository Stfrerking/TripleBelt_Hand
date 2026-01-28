// Version: 2025-07-20
// Description: Updated for CAN-based control — joystick + button data now received over CAN
// Update (2025-12-13): Migrated motor PWM outputs to Teensy_PWM (khoih-prog)
// Update (2025-12-19): vREF migrated BACK to Teensy_PWM for unified PWM control
// Update (2025-12-19): REMOVED nFAULT ISR + related globals/prints for debug isolation
// Update (2026-01-03): Added queued homing sequence (Motor 0 -> Motor 1)

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

// ------------------------------
// Homing sequence (queued)
// ------------------------------
volatile bool homingActive = false;
volatile int homingQueueIndex = 0;

// Which motors to home (in order)
constexpr int HOMING_SEQUENCE[] = {1};
constexpr int HOMING_SEQUENCE_LEN = (int)(sizeof(HOMING_SEQUENCE) / sizeof(HOMING_SEQUENCE[0]));

// ----------- Global ROM limits for Motor Limits ----------- //
long lowROM[6]  = {-100000, -100000, -100000, -100000, -100000, -100000};
long highROM[6] = {100000, 100000, 100000, 100000, 100000, 100000};


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


// =============================
// Homing Constants Setup
// =============================

// Stall detection thresholds [Amps]
// NOTE: Drive–Brake tends to produce higher average IPROPI/current than Drive–Coast at the same mechanical load.
// So we allow separate thresholds for BLUNT (drive–coast) vs PRECISE (drive–brake).
float homingStallA_blunt[6]   = { 0.30f, 0.2f, 0.30f, 0.20f, 0.30f, 0.20f };
float homingStallA_precise[6] = { 0.20f, 0.50f, .20f, 0.50f, 0.20f, 0.50f };

// Ignore current sensing for a short time after entering drive–brake to avoid immediate false "stall" due to braking current spikes.
uint32_t homingBrakeBlankMs[6] = { 50, 50, 50, 50, 50, 50 };

// Stall debounce time [ms]
uint32_t homingStallDebounceMs[6] = { 100, 100, 100, 100, 100, 100 };

// Current sample period during homing [ms]
uint32_t homingSampleMs[6] = { 5, 5, 5, 5, 5, 5 };

// Unstick phase
uint8_t  homingUnstickDuty[6]      = {200, 200, 200, 200, 200, 200 };
uint32_t homingUnstickMs[6]        = { 3000, 3000, 3000, 3000, 3000, 3000 };
int      homingUnstickMinCounts[6] = { 100, 100, 100, 100, 100, 100 };

// Homing timeouts [ms] (Per motor)
// Blunt (drive–coast) search windows for LOW and HIGH ends.
uint32_t homingBluntTimeoutLowMs[6]  = { 12000, 12000, 12000, 12000, 12000, 12000 };
uint32_t homingBluntTimeoutHighMs[6] = { 12000, 12000, 12000, 12000, 12000, 12000 };


// Blunt seek (drive–coast)
uint8_t homingBluntDuty[6] = { 220, 220, 220, 220, 220, 220 };

// Backoff distances [encoder counts]
int homingBluntBackoffCounts[6] = { 1000, 2000, 1000, 2000, 1000, 4000 };
int homingTouchBackoffCounts[6] = { 800, 2000, 800, 2000, 800, 4000 };

// Precise touch-off (drive–brake)
uint8_t  homingPreciseDuty[6]      = { 200, 200, 200, 200, 200, 220 };
uint32_t homingPreciseTimeoutMs[6] = { 1600, 1600, 1600, 1600, 1600, 1600 };
uint8_t  homingTouchRepeats[6]     = { 3, 3, 3, 3, 3, 3 };

// Midpoint move
uint8_t  homingMidDuty[6]        = { 200, 200, 200, 230, 200, 200 };
uint32_t homingMidTimeoutMs[6]   = { 5000, 5000, 5000, 5000, 5000, 5000 };
int      homingMidTolCounts[6]   = { 200, 200, 200, 200, 200, 200 };

// Homing vREF voltage [PWM]
const int homingVoltage[6] = {255, 255, 255, 255, 255, 255};

// Normal (full-scale) vREF duty for regular operation
const int NORMAL_VREF_DUTY = 255;  // 100% duty (0..255)

// Homing speeds [0-100] P will be scaled later
const int homingSpeeds[6] = {50, 50, 50, 50, 50, 50};

// Pullback distances after stall [encoder counts]
const int pullbackCounts[6] = {100, 100, 100, 100, 100, 100};

//Buys time for Cap to settle.
static constexpr uint32_t VREF_SETTLE_MS   = 25;

//Time oveer which we ramp up to full duty. Used to avoid falsely triggering nFAULT due to current surge.
static constexpr uint32_t SOFTSTART_MS     = 60;

//Delays polling of nFAULT, reduced risk of false flagging nFUALT due to current surge on startup.
static constexpr uint32_t STARTUP_BLANK_MS = 35;


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
void driveMotorHomingSoftStart(int index, int direction, int targetDuty, uint32_t rampMs);


static void loadPwmArray(uint8_t *target, const CAN_message_t &msg);

// ----------- Helper forward declarations -----------
static inline uint16_t dutyByteToLevel16(uint8_t dutyByte);
static inline void setMotorDutyByte(Teensy_PWM* pwm, uint8_t dutyByte);
void setVrefDuty(int dutyByte);
static inline float readMotorCurrentA(int index);
static inline void setDriveCoast(int index, int dir, uint8_t duty);
static inline void setDriveBrake(int index, int dir, uint8_t driveDuty);
static bool waitForMotionCounts(int idx, int dir, int minDeltaCounts, uint32_t msMax);

// Homing helper functions
static bool homingStallDetectedTimed(int index,
                                    float stallA,
                                    uint32_t debounceMs,
                                    uint32_t sampleMs,
                                    uint32_t maxMs,
                                    bool &faulted,
                                    uint32_t blankMs = 0);

static void homingBackoffFrom(int index,
                              int approachDir,
                              int counts,
                              uint8_t duty,
                              uint32_t timeoutMs = 750);

static inline long median3(long a, long b, long c);
static long trimmedMean(const long *samples, int n);

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
  // ------------------------------
  // Homing sequence runner (Motor 0 -> Motor 1)
  // ------------------------------
  if (homingRequested && homingActive) {
    homingRequested = false;

    if (homingQueueIndex < HOMING_SEQUENCE_LEN) {
      const int motor = HOMING_SEQUENCE[homingQueueIndex];

      Serial.print("Starting homing for motor ");
      Serial.println(motor);

      homeMotor(motor);
      sendRangeOfMotion();

      homingQueueIndex++;

      // Kick the next motor on the next loop iteration
      homingRequested = true;
    } else {
      homingActive = false;
      Serial.println("Homing sequence complete (motors 0 -> 1)");
    }
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
        // Start the queued homing sequence (0 -> 1) if not already running
        if (!homingActive) {
          homingActive = true;
          homingQueueIndex = 0;
          homingRequested = true;
          Serial.println("Homing sequence requested (motors 0 -> 1)");
        }
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

static bool waitForMotionCounts(int idx, int dir, int minDeltaCounts, uint32_t msMax) {
  const long start = encoders[idx]->read();
  const uint32_t t0 = millis();
  while (millis() - t0 < msMax) {
    const long now = encoders[idx]->read();
    const long d = now - start;
    if ((dir > 0 && d >= minDeltaCounts) || (dir < 0 && d <= -minDeltaCounts)) return true;
  }
  return false;
}

// ------------------------------
// Homing helper functions
// ------------------------------

static bool homingStallDetectedTimed(int index,
                                    float stallA,
                                    uint32_t debounceMs,
                                    uint32_t sampleMs,
                                    uint32_t maxMs,
                                    bool &faulted,
                                    uint32_t blankMs) {
  // NOTE: nFAULT is intentionally ignored for homing (per user request).
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
//Currently Unused. MotorHomingSoftStart is being used instead.
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

// direction: +1 or -1
void driveMotorHomingSoftStart(int index, int direction, int targetDuty, uint32_t rampMs) {
  targetDuty = constrain(targetDuty, 0, 255);

  // Ensure we start from 0 duty
  PWM_A[index] = 0;
  PWM_B[index] = 0;
  controlMotor(index);

  const uint32_t t0 = millis();

  // Linear ramp 0 -> targetDuty over rampMs
  while (millis() - t0 < rampMs) {
    const uint32_t dt = millis() - t0;
    const int duty = (int)((uint32_t)targetDuty * dt / rampMs);

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

  // Hold final target
  if (direction > 0) {
    PWM_A[index] = targetDuty;
    PWM_B[index] = 0;
  } else if (direction < 0) {
    PWM_A[index] = 0;
    PWM_B[index] = targetDuty;
  } else {
    PWM_A[index] = 0;
    PWM_B[index] = 0;
  }

  controlMotor(index);
}

void homeMotor(int index) {
  Serial.print("Homing motor ");
  Serial.println(index);

  // ------------------------------
  // Pull per-motor parameters
  // ------------------------------
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

  const uint8_t  PRECISE_DUTY       = homingPreciseDuty[index];
  const uint32_t PRECISE_TIMEOUT_MS = homingPreciseTimeoutMs[index];
  int            TOUCH_REPEATS      = (int)homingTouchRepeats[index];
  const int      TOUCH_BACKOFF      = homingTouchBackoffCounts[index];

  const uint8_t  MID_DUTY           = homingMidDuty[index];
  const uint32_t MID_TIMEOUT_MS     = homingMidTimeoutMs[index];
  const int      MID_TOL_COUNTS     = homingMidTolCounts[index];

  // Clamp repeats to something sane for stack arrays
  constexpr int MAX_TOUCH = 5;
  TOUCH_REPEATS = constrain(TOUCH_REPEATS, 1, MAX_TOUCH);

  // Define which direction is "LOW" for this motor.
  // Convention used here: LOW = -1, HIGH = +1
  const int DIR_LOW  = -1;
  const int DIR_HIGH = +1;

  // ------------------------------
  // NOTE: nFAULT is intentionally ignored during homing (no pre-clear / no abort-on-fault)
  // ------------------------------

  // ------------------------------
  // Set homing current limit (VREF) and settle
  // ------------------------------
  setVrefDuty(homingVoltage[index]);
  delay(VREF_SETTLE_MS);

  // ------------------------------
  // Homing helper functions are global (stall detect, backoff, median/trimmed mean)
  // ------------------------------

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
    return;
  }

  // Back off to create a known neighborhood
  homingBackoffFrom(index, DIR_LOW, BLUNT_BACKOFF, BLUNT_DUTY);

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
      return;
    }

    lowSamples[k] = encoders[index]->read();

    Serial.print("Sample ");
    Serial.print(k);
    Serial.print(":");
    Serial.println(lowSamples[k]);

    homingBackoffFrom(index, DIR_LOW, TOUCH_BACKOFF, (uint8_t)max(15, (int)PRECISE_DUTY));
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
    return;
  }

  homingBackoffFrom(index, DIR_HIGH, BLUNT_BACKOFF, (uint8_t)max(20, (int)BLUNT_DUTY - 30));

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
      return;
    }

    highSamples[k] = encoders[index]->read();

    Serial.print("Sample ");
    Serial.print(k);
    Serial.print(":");
    Serial.println(highSamples[k]);

    homingBackoffFrom(index, DIR_HIGH, TOUCH_BACKOFF, (uint8_t)max(15, (int)PRECISE_DUTY));
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

  const uint32_t t0 = millis();
  while (millis() - t0 < MID_TIMEOUT_MS) {
    const long pos = encoders[index]->read();
    const long err = mid - pos;

    if (labs(err) <= MID_TOL_COUNTS) break;

    const int dir = (err > 0) ? +1 : -1;
    setDriveCoast(index, dir, MID_DUTY);
    delay(10);
  }

  stopMotor(index);

  // Restore normal VREF
  setVrefDuty(NORMAL_VREF_DUTY);

  Serial.print("Homing complete. low=");
  Serial.print(lowROM[index]);
  Serial.print(" high=");
  Serial.print(highROM[index]);
  Serial.print(" mid=");
  Serial.println(mid);
}

