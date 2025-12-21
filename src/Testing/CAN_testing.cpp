#include <FlexCAN_T4.h>

// ----------- CAN Setup ----------- //
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

// ----------- CAN Message Map (must match Hand) ----------- //
// Brain -> Hand
constexpr uint32_t CAN_ID_CONFIG_CMD   = 0x100;  // requestedConfig
constexpr uint32_t CAN_ID_HOMING_CMD   = 0x120;  // homing start
constexpr uint32_t CAN_ID_PWM_A_CMD    = 0x200;  // PWM_A[0..5]
constexpr uint32_t CAN_ID_PWM_B_CMD    = 0x201;  // PWM_B[0..5]

// Hand -> Brain
constexpr uint32_t CAN_ID_ENCODER_BASE = 0x110;  // + motor index (0..5)
constexpr uint32_t CAN_ID_ROM_BASE     = 0x130;  // + motor index (0..5)

// ----------- IO Pins ----------- //
const int JOY_X_PIN        = 14;  // A0
const int JOY_Y_PIN        = 15;  // A1
const int BUTTON_PIN       = 33;  // Config cycle button
const int HOME_BUTTON_PIN  = 32;  // Button for homing

// ----------- State ----------- //
bool lastButtonState      = false;
bool lastHomeButtonState  = false;

uint8_t currentConfig      = 0;      // 0..3, maps to CONFIG_1..CONFIG_4 on Hand

// Encoders & Range of Motion from Hand
int32_t encoderPositions[6] = {0};
int32_t lowROM[6]           = {0};
int32_t highROM[6]          = {0};

// Brain-side PWM command buffers (to send to Hand)
uint8_t pwmA_cmd[6] = {0};
uint8_t pwmB_cmd[6] = {0};

unsigned long lastPrintTime         = 0;
const unsigned long printIntervalMs = 1000;

// ----------- Function Declarations ----------- //
void handleCAN(const CAN_message_t &msg);
void sendConfig();
void sendHomingRequest();
void computePWM(int rawX, int rawY);      // compute joystick -> per-motor PWM_A/PWM_B
void sendPWM();                           // send current PWM_A/PWM_B over CAN
int joystickToSpeed(int raw);             // helper: raw joystick reading -> signed speed (-255..255)
void setMotorPWM(int motorIndex, int rawJoystickValue);  // helper: pick a motor and a joystick reading

// ----------- CAN Receive Handler ----------- ----------- //
void handleCAN(const CAN_message_t &msg) {
  // Encoder positions: IDs 0x110 .. 0x115, len = 4
  if (msg.id >= CAN_ID_ENCODER_BASE && msg.id < CAN_ID_ENCODER_BASE + 6 && msg.len == 4) {
    int index = msg.id - CAN_ID_ENCODER_BASE;

    int32_t position =
      ((int32_t)msg.buf[0] << 24) |
      ((int32_t)msg.buf[1] << 16) |
      ((int32_t)msg.buf[2] << 8)  |
      (int32_t)msg.buf[3];

    encoderPositions[index] = position;
    return;
  }

  // Range of motion: IDs 0x130 .. 0x135, len = 8 (lowROM, highROM as int32_t)
  if (msg.id >= CAN_ID_ROM_BASE && msg.id < CAN_ID_ROM_BASE + 6 && msg.len == 8) {
    int index = msg.id - CAN_ID_ROM_BASE;

    int32_t low =
      ((int32_t)msg.buf[0] << 24) |
      ((int32_t)msg.buf[1] << 16) |
      ((int32_t)msg.buf[2] << 8)  |
      (int32_t)msg.buf[3];

    int32_t high =
      ((int32_t)msg.buf[4] << 24) |
      ((int32_t)msg.buf[5] << 16) |
      ((int32_t)msg.buf[6] << 8)  |
      (int32_t)msg.buf[7];

    lowROM[index]  = low;
    highROM[index] = high;
    return;
  }

  // Ignore other IDs for now
}

void setup() {
  Serial.begin(115200);

  // CAN init
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.enableMBInterrupts();
  Can0.onReceive(handleCAN);

  pinMode(BUTTON_PIN,      INPUT_PULLUP);
  pinMode(HOME_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Brain: CAN joystick + config + homing controller ready");

  // Optionally send initial config at boot
  sendConfig();
}

void loop() {
  Can0.events();   // Pump CAN RX/TX callbacks

  // ----------- Read Inputs ----------- //
  int rawX = analogRead(JOY_X_PIN);  // 0..1023
  int rawY = analogRead(JOY_Y_PIN);  // 0..1023

  bool currentButtonState     = !digitalRead(BUTTON_PIN);      // Active low
  bool currentHomeButtonState = !digitalRead(HOME_BUTTON_PIN); // Active low

  // Rising edge on config button → cycle config 0..3 and send
  if (currentButtonState && !lastButtonState) {
    currentConfig = (currentConfig + 1) % 4;  // 0,1,2,3
    sendConfig();
  }
  lastButtonState = currentButtonState;

  // Rising edge on home button → send homing request
  if (currentHomeButtonState && !lastHomeButtonState) {
    sendHomingRequest();
  }
  lastHomeButtonState = currentHomeButtonState;

  // ----------- Compute and Send PWM Commands ----------- //
  computePWM(rawX, rawY);   // fill pwmA_cmd[] / pwmB_cmd[] based on joystick
  sendPWM();                // package and transmit them over CAN

  // ----------- Debug Print ----------- //
  unsigned long now = millis();
  if (now - lastPrintTime >= printIntervalMs) {
    lastPrintTime = now;

    Serial.print("Joystick raw: X=");
    Serial.print(rawX);
    Serial.print(" Y=");
    Serial.print(rawY);
    Serial.print("  Config=");
    Serial.println(currentConfig);

    Serial.print("Encoders: ");
    for (int i = 0; i < 6; i++) {
      Serial.print("E");
      Serial.print(i);
      Serial.print("=");
      Serial.print(encoderPositions[i]);
      Serial.print("  ");
    }
    Serial.println();

    Serial.print("ROM: ");
    for (int i = 0; i < 6; i++) {
      Serial.print("[");
      Serial.print(lowROM[i]);
      Serial.print(",");
      Serial.print(highROM[i]);
      Serial.print("] ");
    }
    Serial.println();
  }

  delay(10);  // Small delay to keep loop sane
}

// ----------- Send Config to Hand ----------- //
void sendConfig() {
  CAN_message_t msg;
  msg.id  = CAN_ID_CONFIG_CMD;
  msg.len = 1;
  msg.buf[0] = currentConfig;  // 0..3

  Can0.write(msg);

  Serial.print("Sent config requestedConfig = ");
  Serial.println(currentConfig);
}

// ----------- Send Homing Request ----------- //
void sendHomingRequest() {
  CAN_message_t msg;
  msg.id  = CAN_ID_HOMING_CMD;
  msg.len = 1;
  msg.buf[0] = 1;   // 1 = start homing sequence

  Can0.write(msg);

  Serial.println("Sent homing request");
}

// ----------- Helper: convert one joystick axis to signed speed ----------- //
// raw: 0..1023 from analogRead
// return: -255..+255 (negative = reverse, positive = forward, 0 = stop)
int joystickToSpeed(int raw) {
  const int CENTER   = 512;   // ideal center of the joystick
  const int DEADZONE = 50;    // region around center where we treat it as 0
  const int MAX_PWM  = 255;   // max command we send to the DRV8874 side

  // How far away from center are we? (negative = one way, positive = the other)
  int offset = raw - CENTER;

  // Inside deadzone -> treat as zero
  if (offset > -DEADZONE && offset < DEADZONE) {
    return 0;
  }

  // Work with magnitude (how far from center), then re-apply sign at the end
  int magnitude = abs(offset) - DEADZONE;          // remove deadzone
  int maxOffset = CENTER - DEADZONE;               // ~462

  // Clamp magnitude into [0, maxOffset]
  if (magnitude < 0) {
    magnitude = 0;
  }
  if (magnitude > maxOffset) {
    magnitude = maxOffset;
  }

  // Scale magnitude linearly into [0, MAX_PWM]
  long scaled = (long)magnitude * MAX_PWM / maxOffset;  // use long to avoid overflow
  int pwm = (int)scaled;

  // Put the sign back
  if (offset < 0) {
    pwm = -pwm;
  }

  return pwm;
}

// ----------- Compute PWM_A / PWM_B from joystick ----------- //
// This function now just clears all commands and then calls
// setMotorPWM(...) for whichever motors you want to control.

// Decide PWM for a *single* motor, given:
//   - motorIndex: which motor (0..5)
//   - rawJoystickValue: 0..1023 from analogRead
// This uses joystickToSpeed() to get a signed speed (-255..+255)
// and then maps that to the DRV8874's A/B side for that motor.
void setMotorPWM(int motorIndex, int rawJoystickValue) {
  // Safety check: ignore invalid indices
  if (motorIndex < 0 || motorIndex >= 6) {
    return;
  }

  int speed = joystickToSpeed(rawJoystickValue);  // -255..+255

  if (speed > 0) {
    // Forward direction: A side gets PWM, B side off
    pwmA_cmd[motorIndex] = (uint8_t)speed;
    pwmB_cmd[motorIndex] = 0;
  } else if (speed < 0) {
    // Reverse direction: B side gets PWM, A side off
    pwmA_cmd[motorIndex] = 0;
    pwmB_cmd[motorIndex] = (uint8_t)(-speed);  // make positive
  } else {
    // Deadzone / zero: motor off
    pwmA_cmd[motorIndex] = 0;
    pwmB_cmd[motorIndex] = 0;
  }
}

// High-level: clear everything, then specify which motors
// are controlled by which joystick readings.
void computePWM(int rawX, int rawY) {
  // 1) Clear command buffers for all 6 motors
  for (int i = 0; i < 6; i++) {
    pwmA_cmd[i] = 0;
    pwmB_cmd[i] = 0;
  }

  // Later, we can call setMotorPWM() from anywhere with maybe custom values for invsere kinematics
  // instead of just pulling RawX or RawY

  setMotorPWM(0, rawX);
  setMotorPWM(1, rawY); 
  setMotorPWM(2, rawX);
  setMotorPWM(3, rawY);
  setMotorPWM(4, rawX);
  setMotorPWM(5, rawY);
  
}

// ----------- Send current PWM_A / PWM_B over CAN ----------- //
void sendPWM() {
  // Build and send PWM_A message
  CAN_message_t msgA;
  msgA.id  = CAN_ID_PWM_A_CMD;
  msgA.len = 6;  // 6 motors
  for (int i = 0; i < 6; i++) {
    msgA.buf[i] = pwmA_cmd[i];
  }
  Can0.write(msgA);

  // Build and send PWM_B message
  CAN_message_t msgB;
  msgB.id  = CAN_ID_PWM_B_CMD;
  msgB.len = 6;
  for (int i = 0; i < 6; i++) {
    msgB.buf[i] = pwmB_cmd[i];
  }
  Can0.write(msgB);
}
