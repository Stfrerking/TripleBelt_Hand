#include <FlexCAN_T4.h>
#include <Encoder.h>  // Hardware quadrature decoding library

// Use CAN3
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

const int DIR_A_PIN = 25;
const int DIR_B_PIN = 26;
const int DEADZONE = 50;  // Adjust to prevent motor twitching near center

// Encoder on pins 0 (A) and 1 (B)
Encoder motorEncoder(0, 1);
long lastEncoderPosition = 0;

elapsedMillis encoderReportTimer;  // Tracks time since last encoder transmission
const unsigned int REPORT_INTERVAL = 200; // ms

void handleCAN(const CAN_message_t &msg) {
  if (msg.id == 0x100 && msg.len == 6) {
    int joyX = (msg.buf[0] << 8) | msg.buf[1];
    int joyY = (msg.buf[2] << 8) | msg.buf[3];
    int buttonCounter = (msg.buf[4] << 8) | msg.buf[5];

    Serial.print("Joystick X: "); Serial.print(joyX);
    Serial.print(" | Y: "); Serial.print(joyY);
    Serial.print(" | Button Count: "); Serial.print(buttonCounter);

    // Motor direction control
    int center = 512;  // Change to 2048 if using 12-bit ADC
    if (joyY > center + DEADZONE) {
      digitalWrite(DIR_A_PIN, HIGH);
      digitalWrite(DIR_B_PIN, LOW);  // Forward
    } else if (joyY < center - DEADZONE) {
      digitalWrite(DIR_A_PIN, LOW);
      digitalWrite(DIR_B_PIN, HIGH); // Backward
    } else {
      digitalWrite(DIR_A_PIN, LOW);
      digitalWrite(DIR_B_PIN, LOW);  // Brake/stop
    }

    // Read encoder position
    long currentEncoderPosition = motorEncoder.read();
    long encoderDelta = currentEncoderPosition - lastEncoderPosition;
    lastEncoderPosition = currentEncoderPosition;

    Serial.print(" | Encoder: "); Serial.print(currentEncoderPosition);
    Serial.print(" (Δ"); Serial.print(encoderDelta); Serial.println(")");
  } else {
    Serial.println("Unknown message received.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait for serial connection

  pinMode(DIR_A_PIN, OUTPUT);
  pinMode(DIR_B_PIN, OUTPUT);
  digitalWrite(DIR_A_PIN, LOW);
  digitalWrite(DIR_B_PIN, LOW);

  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.setMaxMB(16);
  Can3.enableMBInterrupts();
  Can3.onReceive(handleCAN);
  Can3.mailboxStatus();

  Serial.println("CAN3 receiver with motor + encoder control listening...");
}

void loop() {
  // Periodically send encoder position to the brain
  if (encoderReportTimer >= REPORT_INTERVAL) {
    encoderReportTimer = 0;

    long position = motorEncoder.read();

    CAN_message_t feedback;
    feedback.id = 0x101;       // Feedback ID
    feedback.len = 4;
    feedback.buf[0] = (position >> 24) & 0xFF;
    feedback.buf[1] = (position >> 16) & 0xFF;
    feedback.buf[2] = (position >> 8) & 0xFF;
    feedback.buf[3] = position & 0xFF;

    Can3.write(feedback);

    Serial.print("Sent encoder position: ");
    Serial.println(position);
  }
}

// brain code

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const int JOY_X_PIN = 14;  // A0
const int JOY_Y_PIN = 15;  // A1
const int BUTTON_PIN = 33; // Digital input

int buttonCounter = 0;
bool lastButtonState = false;

// CAN receive handler
void handleCAN(const CAN_message_t &msg) {
  if (msg.id == 0x101 && msg.len == 4) {
    long encoderPosition =
      ((long)msg.buf[0] << 24) |
      ((long)msg.buf[1] << 16) |
      ((long)msg.buf[2] << 8) |
      (long)msg.buf[3];

    Serial.print("Received Encoder Position: ");
    Serial.println(encoderPosition);
  }
}

void setup() {
Can0.begin();
Can0.setBaudRate(500000);
Can0.enableMBInterrupts();  // <-- Add this
Can0.onReceive(handleCAN);

  Can0.onReceive(handleCAN);  // Register receive callback

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Use pullup if button goes LOW when pressed

  Serial.println("CAN transmitter ready");
}

void loop() {
  int joyX = 1023 - analogRead(JOY_X_PIN);  // 0 to 1023 or 4095 depending on resolution
  int joyY = analogRead(JOY_Y_PIN);
  bool currentButtonState = !digitalRead(BUTTON_PIN); // Invert because of pullup

  // Detect rising edge: was not pressed, now pressed
  if (currentButtonState && !lastButtonState) {
    buttonCounter++;
  }

  lastButtonState = currentButtonState;

  CAN_message_t msg;
  msg.id = 0x100;
  msg.len = 6;

  // Package joystick and counter data into 6 bytes
  msg.buf[0] = joyX >> 8;
  msg.buf[1] = joyX & 0xFF;
  msg.buf[2] = joyY >> 8;
  msg.buf[3] = joyY & 0xFF;
  msg.buf[4] = (buttonCounter >> 8) & 0xFF;  // High byte of counter
  msg.buf[5] = buttonCounter & 0xFF;         // Low byte of counter

  Can0.write(msg);

  Serial.print("Sent: X="); Serial.print(joyX);
  Serial.print(" Y="); Serial.print(joyY);
  Serial.print(" Count="); Serial.println(buttonCounter);

  delay(100); // Send every 100 ms
}
