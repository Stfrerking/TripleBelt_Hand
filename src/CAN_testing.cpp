// UPDATED Brain firmware
// Purpose: Replace per-motor PWM streaming with absolute encoder position setpoints
// Architecture: Brain computes behavior (parallel grip / IK later)
//               Hand is a servo node that accepts absolute pos targets
//
// CAN byte order rule (THIS FILE):
// - All int32 values on CAN are little-endian (LSB first).

#include <FlexCAN_T4.h>

// ----------- CAN Setup ----------- //
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

// ----------- CAN Message Map (must match Hand) ----------- //
// Brain -> Hand
constexpr uint32_t CAN_ID_HOMING_CMD   = 0x120;  // homing start
constexpr uint32_t CAN_ID_SET_SERVO_POS = 0x220; // servo0, servo1 angle targets (int32 LE)

constexpr uint32_t CAN_ID_SET_POS_01   = 0x210;  // pos0, pos1 (int32 LE)
constexpr uint32_t CAN_ID_SET_POS_23   = 0x211;  // pos2, pos3
constexpr uint32_t CAN_ID_SET_POS_45   = 0x212;  // pos4, pos5

// Hand -> Brain
constexpr uint32_t CAN_ID_ENCODER_BASE = 0x110;  // + motor index (0..5)
constexpr uint32_t CAN_ID_ROM_BASE     = 0x130;  // + motor index (0..5)
constexpr uint32_t CAN_ID_CURRENT_BASE = 0x140;  // + motor index (0..5), current in mA (int32 LE)

// ----------- IO Pins ----------- //
const int JOY_X_PIN        = 22;  // A0
const int JOY_Y_PIN        = 23;  // A1
const int BUTTON_PIN       = 1;  // Servo preset cycle button
const int HOME_BUTTON_PIN  = 7;  // Button for homing

// ----------- State ----------- //
bool lastButtonState      = false;
bool lastHomeButtonState  = false;

// Encoder + ROM data from Hand
int32_t encoderPositions[6] = {0};
int32_t lowROM[6]           = {0};
int32_t highROM[6]          = {0};
int32_t motorCurrents_mA[6] = {0};

// Absolute target positions to send to Hand
int32_t posTarget[6] = {0};
struct ServoPreset {
  const char *name;
  int32_t servoAnglesDeg[2];
};

const ServoPreset SERVO_PRESETS[] = {
  {"One-sided Grip", {5, 210}},
  {"Pinch Grip", {102, 98}},
  {"Claw Grip", {140, 45}},
  {"Coffee Cup Grip", {210, 5}},
};
constexpr uint8_t SERVO_PRESET_COUNT = sizeof(SERVO_PRESETS) / sizeof(SERVO_PRESETS[0]);

uint8_t currentServoPreset = 0;
int32_t servoTargetDeg[2] = {
  SERVO_PRESETS[0].servoAnglesDeg[0],
  SERVO_PRESETS[0].servoAnglesDeg[1],
};

unsigned long lastPrintTime         = 0;
const unsigned long printIntervalMs = 1000;

// ----------- Function Declarations ----------- //
void sendHomingRequest();
void sendPositionTargets();
void applyServoPreset(uint8_t presetIndex);
void sendServoPositions();

// Joystick → delta counts per loop (VERY conservative to start)
int32_t joystickToDelta(int raw) {
  const int CENTER   = 512;
  const int DEADZONE = 50;
  const int MAX_DPOS = 100;   // counts per loop @ full deflection

  int offset = raw - CENTER;
  if (offset > -DEADZONE && offset < DEADZONE) return 0;

  int mag = abs(offset) - DEADZONE;
  int maxOffset = CENTER - DEADZONE;
  if (mag < 0) mag = 0;
  if (mag > maxOffset) mag = maxOffset;

  int32_t delta = (int32_t)mag * MAX_DPOS / maxOffset;
  if (offset < 0) delta = -delta;
  return delta;
}

// ----------- Little-endian pack/unpack helpers ----------- //
static inline void pack_i32_le(uint8_t *b, int32_t v) {
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  b[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline int32_t unpack_i32_le(const uint8_t *b) {
  return (int32_t)(
    ((uint32_t)b[0])        |
    ((uint32_t)b[1] << 8)   |
    ((uint32_t)b[2] << 16)  |
    ((uint32_t)b[3] << 24)
  );
}

// ----------- CAN Receive Handler ----------- //
void handleCAN(const CAN_message_t &msg) {
  // Encoder positions: IDs 0x110 .. 0x115 (int32 LE)
  if (msg.id >= CAN_ID_ENCODER_BASE && msg.id < CAN_ID_ENCODER_BASE + 6 && msg.len == 4) {
    int i = msg.id - CAN_ID_ENCODER_BASE;
    encoderPositions[i] = unpack_i32_le(msg.buf);
    return;
  }

  // ROM limits: IDs 0x130 .. 0x135 (lo=int32 LE, hi=int32 LE)
  if (msg.id >= CAN_ID_ROM_BASE && msg.id < CAN_ID_ROM_BASE + 6 && msg.len == 8) {
    int i = msg.id - CAN_ID_ROM_BASE;
    lowROM[i]  = unpack_i32_le(&msg.buf[0]);
    highROM[i] = unpack_i32_le(&msg.buf[4]);
    return;
  }

  // Motor currents: IDs 0x140 .. 0x145 (mA as int32 LE)
  if (msg.id >= CAN_ID_CURRENT_BASE && msg.id < CAN_ID_CURRENT_BASE + 6 && msg.len == 4) {
    int i = msg.id - CAN_ID_CURRENT_BASE;
    motorCurrents_mA[i] = unpack_i32_le(msg.buf);
    return;
  }
}

void setup() {
  Serial.begin(115200);

  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.enableMBInterrupts();
  Can0.onReceive(handleCAN);

  pinMode(BUTTON_PIN,      INPUT_PULLUP);
  pinMode(HOME_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Brain: Absolute position controller ready");
  sendServoPositions();
}

void loop() {
  Can0.events();

  int rawX = analogRead(JOY_X_PIN);
  int rawY = analogRead(JOY_Y_PIN);

  bool btn     = !digitalRead(BUTTON_PIN);
  bool homeBtn = !digitalRead(HOME_BUTTON_PIN);

  if (btn && !lastButtonState) {
    currentServoPreset = (currentServoPreset + 1) % SERVO_PRESET_COUNT;
    applyServoPreset(currentServoPreset);
  }
  lastButtonState = btn;

  if (homeBtn && !lastHomeButtonState) {
    sendHomingRequest();
  }
  lastHomeButtonState = homeBtn;

  // ----------- Compute absolute targets ----------- //
  int32_t dx = joystickToDelta(rawX);
  int32_t dy = joystickToDelta(rawY);

  // Simple test behavior: X drives even motors, Y drives odd motors
  for (int i = 0; i < 6; i++) {
    int32_t delta = (i % 2 == 0) ? dx : dy;
    posTarget[i] += delta;  // NO ROM clamping in Brain (Hand will clamp)
  }

  sendPositionTargets();
  sendServoPositions();

  // ----------- Debug ----------- //
  unsigned long now = millis();
  if (now - lastPrintTime >= printIntervalMs) {
    lastPrintTime = now;
    Serial.print("Targets: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(posTarget[i]); Serial.print(" ");
    }
    Serial.println();

    Serial.print("Enc Positions: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(encoderPositions[i]); Serial.print(" ");
    }
    Serial.println();

    Serial.print("Motor Currents A: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(motorCurrents_mA[i] / 1000.0f, 3); Serial.print(" ");
    }
    Serial.println();

    Serial.print("Servo Targets Deg: ");
    Serial.print(SERVO_PRESETS[currentServoPreset].name);
    Serial.print(": ");
    Serial.print(servoTargetDeg[0]); Serial.print(" ");
    Serial.println(servoTargetDeg[1]);
  }

  delay(10);
}

// ----------- CAN Send Helpers ----------- //
void applyServoPreset(uint8_t presetIndex) {
  if (presetIndex >= SERVO_PRESET_COUNT) return;

  servoTargetDeg[0] = SERVO_PRESETS[presetIndex].servoAnglesDeg[0];
  servoTargetDeg[1] = SERVO_PRESETS[presetIndex].servoAnglesDeg[1];
  sendServoPositions();

  Serial.print("Servo Preset Sent: ");
  Serial.print(SERVO_PRESETS[presetIndex].name);
  Serial.print(" (");
  Serial.print(servoTargetDeg[0]);
  Serial.print(", ");
  Serial.print(servoTargetDeg[1]);
  Serial.println(")");
}

void sendServoPositions() {
  CAN_message_t msg;
  msg.id = CAN_ID_SET_SERVO_POS;
  msg.len = 8;
  pack_i32_le(&msg.buf[0], servoTargetDeg[0]);
  pack_i32_le(&msg.buf[4], servoTargetDeg[1]);
  Can0.write(msg);
}

void sendHomingRequest() {
  CAN_message_t msg;
  msg.id = CAN_ID_HOMING_CMD;
  msg.len = 1;
  msg.buf[0] = 1;
  Can0.write(msg);
  Serial.println("Homing Request Sent");
}

void sendPositionTargets() {
  CAN_message_t m;
  m.len = 8;

  m.id = CAN_ID_SET_POS_01;
  pack_i32_le(m.buf + 0, posTarget[0]);
  pack_i32_le(m.buf + 4, posTarget[1]);
  Can0.write(m);

  m.id = CAN_ID_SET_POS_23;
  pack_i32_le(m.buf + 0, posTarget[2]);
  pack_i32_le(m.buf + 4, posTarget[3]);
  Can0.write(m);

  m.id = CAN_ID_SET_POS_45;
  pack_i32_le(m.buf + 0, posTarget[4]);
  pack_i32_le(m.buf + 4, posTarget[5]);
  Can0.write(m);
}
