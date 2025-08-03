#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const int JOY_X_PIN = 14;  // A0
const int JOY_Y_PIN = 15;  // A1
const int BUTTON_PIN = 33; // Digital input
const int HOME_BUTTON_PIN = 32; // Button for homing

int buttonCounter = 0;
bool lastButtonState = false;
bool lastHomeButtonState = false;

int32_t encoderPositions[6] = {0};  // Stores positions for encoders 0 to 5

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;

void handleCAN(const CAN_message_t &msg) {
  if (msg.len == 4 && msg.id >= 0x110 && msg.id <= 0x115) {
    int encoderIndex = msg.id - 0x110;

    int32_t position =
      ((int32_t)msg.buf[0] << 24) |
      ((int32_t)msg.buf[1] << 16) |
      ((int32_t)msg.buf[2] << 8) |
      (int32_t)msg.buf[3];

    encoderPositions[encoderIndex] = position;
  }
}

void setup() {
  Serial.begin(115200);
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.enableMBInterrupts();
  Can0.onReceive(handleCAN);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(HOME_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("CAN transmitter and encoder listener ready");
}

void loop() {
  int joyX = 1023 - analogRead(JOY_X_PIN);
  int joyY = analogRead(JOY_Y_PIN);
  bool currentButtonState = !digitalRead(BUTTON_PIN);
  bool currentHomeButtonState = !digitalRead(HOME_BUTTON_PIN);

  if (currentButtonState && !lastButtonState) {
    buttonCounter++;
  }
  lastButtonState = currentButtonState;

  CAN_message_t msg;
  msg.id = 0x100;
  msg.len = 6;
  msg.buf[0] = joyX >> 8;
  msg.buf[1] = joyX & 0xFF;
  msg.buf[2] = joyY >> 8;
  msg.buf[3] = joyY & 0xFF;
  msg.buf[4] = (buttonCounter >> 8) & 0xFF;
  msg.buf[5] = buttonCounter & 0xFF;

  Can0.write(msg);

  // Send homing request if home button is pressed
  if (currentHomeButtonState && !lastHomeButtonState) {
    CAN_message_t homeMsg;
    homeMsg.id = 0x120;  // Unique ID for homing command
    homeMsg.len = 1;
    homeMsg.buf[0] = 1;
    Can0.write(homeMsg);
    Serial.println("Sent homing request");
  }
  lastHomeButtonState = currentHomeButtonState;

  unsigned long now = millis();
  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;

    Serial.print("Sent: X="); Serial.print(joyX);
    Serial.print(" Y="); Serial.print(joyY);
    Serial.print(" Count="); Serial.println(buttonCounter);

    for (int i = 0; i < 6; i++) {
      Serial.print("Enc"); Serial.print(i);
      Serial.print(": "); Serial.print(encoderPositions[i]);
      Serial.print("\t");
    }
    Serial.println();
  }

  delay(100);
}
