// Version: 2025-07-20
// Description: Updated for CAN-based control — joystick + button data now received over CAN

#include <PWMServo.h>
#include <Arduino.h>
#include <Encoder.h>
#include <FlexCAN_T4.h>

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
#define MOTOR0_PWM 4
#define MOTOR0_DIR_A 25
#define MOTOR0_DIR_B 26
#define MOTOR1_PWM 5
#define MOTOR1_DIR_A 27
#define MOTOR1_DIR_B 15
#define MOTOR2_PWM 10
#define MOTOR2_DIR_A 14
#define MOTOR2_DIR_B 32
#define MOTOR3_PWM 11
#define MOTOR3_DIR_A 13
#define MOTOR3_DIR_B 41
#define MOTOR4_PWM 12
#define MOTOR4_DIR_A 39
#define MOTOR4_DIR_B 40
#define MOTOR5_PWM 24
#define MOTOR5_DIR_A 37
#define MOTOR5_DIR_B 38

// ----------- Current Sense Pins ----------- //
#define MOTOR0_SEN 16
#define MOTOR1_SEN 17
#define MOTOR2_SEN 18
#define MOTOR3_SEN 19
#define MOTOR4_SEN 20
#define MOTOR5_SEN 21

// ----------- Servo Pins ----------- //
#define SERVO0 28
#define SERVO1 29

// ----------- CAN Declaration ----------- //
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

volatile bool configChangeRequested = false;

// ----------- Global ROM limits for Motor Limits ----------- //
long lowROM[6]  = {-100000, -100000, -100000, -100000, -100000, -100000};
long highROM[6] = {100000, 100000, 100000, 100000, 100000, 100000};

#define HOMING_TIMEOUT 8000
Encoder* encoders[6] = { &encoder0, &encoder1, &encoder2, &encoder3, &encoder4, &encoder5 };
const int motorPWM[6]   = {MOTOR0_PWM, MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM, MOTOR5_PWM};
const int motorDIR_A[6] = {MOTOR0_DIR_A, MOTOR1_DIR_A, MOTOR2_DIR_A, MOTOR3_DIR_A, MOTOR4_DIR_A, MOTOR5_DIR_A};
const int motorDIR_B[6] = {MOTOR0_DIR_B, MOTOR1_DIR_B, MOTOR2_DIR_B, MOTOR3_DIR_B, MOTOR4_DIR_B, MOTOR5_DIR_B};
const int motorSEN[6]   = {MOTOR0_SEN, MOTOR1_SEN, MOTOR2_SEN, MOTOR3_SEN, MOTOR4_SEN, MOTOR5_SEN};

// Stall current thresholds [A]
float stallThresholds[6] = {.3, .3, .3, .3, .3, .3};

// Homing speeds [0-100] PWM will be scalled later
const int homingSpeeds[6] = {100, 80, 100, 80, 100, 80};

// Backoff durations [ms]
const int backoffTimes[6] = {50, 100, 50, 100, 50, 100};

// Pullback distances after stall [encoder counts]
const int pullbackCounts[6] = {10, 10, 10, 10, 10, 10};

// Stall debounce [ms] — time without motion before we consider it stuck
const unsigned long stallDebounce[6] = {60, 80, 60, 80, 60, 80};

int processedX = 0;
int processedY = 0;

PWMServo servo0;
PWMServo servo1;

// Function declarations 
void sendEncoderPositions();
void homeMotors();
void nextState();
void handleState();
void printEncoderData();
void printAllROMs();
void controlMotor(int index, int speed, long lowROM, long highROM);


//Declaration for finger configurations driven by servos
enum HandState { CONFIG_1, CONFIG_2, CONFIG_3, CONFIG_4 };
HandState currentState = CONFIG_1;
uint16_t lastButtonCounter = 0;

void configChangeISR() {
  configChangeRequested = true;
}

float lerp(float x, float x1, float x2, float y1, float y2) {
  return y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
}

void handleCAN(const CAN_message_t &msg) {
  if (msg.id == 0x100 && msg.len == 6) {
    int joyX = ((int)msg.buf[0] << 8) | msg.buf[1];
    int joyY = ((int)msg.buf[2] << 8) | msg.buf[3];
    uint16_t buttonCounter = ((uint16_t)msg.buf[4] << 8) | msg.buf[5];

    processedX = (int)lerp((float)joyX, 0.0, 1023.0, -100.0, 100.0);
    processedY = (int)lerp((float)joyY, 0.0, 1023.0, -100.0, 100.0);

    processedX = abs(processedX) > 10 ? processedX : 0;
    processedY = abs(processedY) > 10 ? processedY : 0;

    if (buttonCounter != lastButtonCounter) {
      configChangeRequested = true;
      lastButtonCounter = buttonCounter;
    }
  } else if (msg.id == 0x120 && msg.len == 1 && msg.buf[0] == 1) {
    Serial.println("Homing request received via CAN");
    homeMotors();
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  for (int i = 0; i < 6; i++) {
    pinMode(motorPWM[i], OUTPUT);
    pinMode(motorDIR_A[i], OUTPUT);
    pinMode(motorDIR_B[i], OUTPUT);
  }

  servo0.attach(SERVO0);
  servo1.attach(SERVO1);

  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.enableMBInterrupts();
  Can3.onReceive(handleCAN);

  Serial.println("System Initialized (CAN-based)");
  homeMotors();
  handleState();
}

void loop() {
  sendEncoderPositions();
  Can3.events();

  if (configChangeRequested) {
    configChangeRequested = false;
    nextState();  
  }

  switch (currentState) {
    case CONFIG_1:
      for (int i = 0; i < 6; i++) {
        controlMotor(i, (i % 2 == 0) ? processedX : processedY, lowROM[i], highROM[i]);
      }
      break;
    case CONFIG_2:
      controlMotor(0, processedX, lowROM[0], highROM[0]);
      controlMotor(1, processedY, lowROM[1], highROM[1]);
      break;
    case CONFIG_3:
      controlMotor(2, processedX, lowROM[2], highROM[2]);
      controlMotor(3, processedY, lowROM[3], highROM[3]);
      break;
    case CONFIG_4:
      controlMotor(4, processedX, lowROM[4], highROM[4]);
      controlMotor(5, processedY, lowROM[5], highROM[5]);
      break;
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    Serial.print("X: "); Serial.print(processedX); Serial.print("\tY: "); Serial.println(processedY);
    printEncoderData();

    for (int i = 0; i < 6; i++) {
      int raw = analogRead(motorSEN[i]);
      float voltage = raw * (3.3 / 4095.0);
      float current = voltage / 0.5;
      Serial.print("M"); Serial.print(i);
      Serial.print(": ADC="); Serial.print(raw);
      Serial.print("  I="); Serial.print(current, 3);
      Serial.print(" A   ");
    }
    Serial.println();
    lastPrint = millis();
  }
}

// Remaining functions retained below

void nextState() {
  currentState = static_cast<HandState>((currentState + 1) % 4);
  Serial.print("Transitioned to state: ");
  Serial.println(currentState);
  handleState();
}

void handleState() {
  switch (currentState) {
    case CONFIG_1:
      Serial.println("State 1: One-sided Grip");
      servo0.write(160);
      servo1.write(20);
      break;
    case CONFIG_2:
      Serial.println("State 2: Pinch Grip");
      servo0.write(90);
      servo1.write(90);
      break;
    case CONFIG_3:
      Serial.println("State 3: Claw Grip");
      servo0.write(45);
      servo1.write(135);
      break;
    case CONFIG_4:
      Serial.println("State 4: Coffee Cup Grip");
      servo0.write(15);
      servo1.write(160);
      break;
  }
}

void controlMotor(int index, int speed, long lowROM, long highROM) {
  long pos = encoders[index]->read();
  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0;

  if ((target > 0 && normalizedPos >= 1.0) || (target < 0 && normalizedPos <= 0.0)) {
    analogWrite(motorPWM[index], 0);
    return;
  }

  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  digitalWrite(motorDIR_A[index], speed < 0);
  digitalWrite(motorDIR_B[index], speed > 0);
  analogWrite(motorPWM[index], pwmValue);
}

void printEncoderData() {
  Serial.print("Enc0: "); Serial.print(encoder0.read()); Serial.print("	");
  Serial.print("Enc1: "); Serial.print(encoder1.read()); Serial.print("	");
  Serial.print("Enc2: "); Serial.print(encoder2.read()); Serial.print("	");
  Serial.print("Enc3: "); Serial.print(encoder3.read()); Serial.print("	");
  Serial.print("Enc4: "); Serial.print(encoder4.read()); Serial.print("	");
  Serial.print("Enc5: "); Serial.println(encoder5.read());
}

void homeMotor(int index, int currentSensePin) {
  Serial.println();
  Serial.print("==== Starting homing for motor ");
  Serial.print(index);
  Serial.println(" ====");

  long prevPos = encoders[index]->read();
  unsigned long lastMoveTime = millis();
  unsigned long startTime = millis();
  float alpha = 0.2;
  float currentFiltered = 0;

  controlMotor(index, -homingSpeeds[index], -100000, 100000);
  Serial.println("[Low-end Homing Phase]");

  while (millis() - startTime < HOMING_TIMEOUT) {
    long currPos = encoders[index]->read();
    bool encoderMoved = (currPos != prevPos);
    unsigned long now = millis();
    unsigned long timeSinceMove = now - lastMoveTime;

    if (encoderMoved) {
      lastMoveTime = now;
      prevPos = currPos;
    }

    int rawADC = analogRead(currentSensePin);
    float voltage = rawADC * (3.3 / 4095.0);
    float current = voltage / 0.5;
    currentFiltered = alpha * current + (1 - alpha) * currentFiltered;

    bool stuck = timeSinceMove > stallDebounce[index];
    bool overCurrent = currentFiltered > stallThresholds[index];

    if (stuck && overCurrent) {
      Serial.println("Low-end stall detected!");
      break;
    }
    delay(20);
  }

  controlMotor(index, 0, lowROM[index], highROM[index]);
  controlMotor(index, 100, -100000, 100000);
  delay(backoffTimes[index]);
  controlMotor(index, 0, lowROM[index], highROM[index]);

  lowROM[index] = encoders[index]->read() + pullbackCounts[index];
  encoders[index]->write(0);
  Serial.print("Set lowROM["); Serial.print(index); Serial.print("]: ");
  Serial.println(lowROM[index]);

  startTime = millis();
  prevPos = encoders[index]->read();
  lastMoveTime = millis();
  controlMotor(index, homingSpeeds[index], -100000, 100000);
  Serial.println("[High-end Homing Phase]");

  while (millis() - startTime < HOMING_TIMEOUT) {
    long currPos = encoders[index]->read();
    bool encoderMoved = (currPos != prevPos);
    unsigned long now = millis();
    unsigned long timeSinceMove = now - lastMoveTime;

    if (encoderMoved) {
      lastMoveTime = now;
      prevPos = currPos;
    }

    int rawADC = analogRead(currentSensePin);
    float voltage = rawADC * (3.3 / 4095.0);
    float current = voltage / 0.5;
    currentFiltered = alpha * current + (1 - alpha) * currentFiltered;

    bool stuck = timeSinceMove > stallDebounce[index];
    bool overCurrent = currentFiltered > stallThresholds[index];

    if (stuck && overCurrent) {
      Serial.println("High-end stall detected!");
      break;
    }
    delay(20);
  }

  controlMotor(index, 0, lowROM[index], highROM[index]);
  controlMotor(index, -100, lowROM[index], highROM[index]);
  delay(backoffTimes[index]);
  controlMotor(index, 0, lowROM[index], highROM[index]);

  highROM[index] = encoders[index]->read() - pullbackCounts[index];
  Serial.print("Set highROM["); Serial.print(index); Serial.print("]: ");
  Serial.println(highROM[index]);

  long midpoint = (lowROM[index] + highROM[index]) / 2;
  long range = highROM[index] - lowROM[index];
  Serial.print("Calculated midpoint: "); Serial.println(midpoint);
  Serial.print("ROM Range: "); Serial.println(range);

  if (range < 500) {
    Serial.println("Warning: ROM range too small, skipping midpoint move!");
    return;
  }

  startTime = millis();
  long lastPos = encoders[index]->read();
  Serial.println("[Midpoint Move Phase]");

  while (abs(encoders[index]->read() - midpoint) > 10 && millis() - startTime < HOMING_TIMEOUT) {
    long currentPos = encoders[index]->read();
    int error = midpoint - currentPos;
    int speed = constrain(map(abs(error), 0, range / 2, 80, homingSpeeds[index]), 70, homingSpeeds[index]);
    int direction = (error > 0) ? 1 : -1;
    controlMotor(index, direction * speed, lowROM[index], highROM[index]);
    delay(20);

    if (abs(currentPos - lastPos) < 3) {
      delay(200);
      long retryPos = encoders[index]->read();
      if (abs(retryPos - currentPos) < 3) {
        Serial.println("Warning: Stuck during midpoint move, exiting.");
        break;
      }
    }
    lastPos = currentPos;
  }
  controlMotor(index, 0, lowROM[index], highROM[index]);
  Serial.println("Midpoint reached successfully.");
  Serial.print("==== Finished homing motor ");
  Serial.print(index);
  Serial.println(" ====");
}

void homeMotors() {
  for (int i = 0; i < 0; i++) {
    homeMotor(i, motorSEN[i]);
  }
  printAllROMs();
}

void sendEncoderPositions() {
  for (int i = 0; i < 6; i++) {
    CAN_message_t msg;
    msg.id = 0x110 + i;
    msg.len = 4;
    long pos = encoders[i]->read();

    msg.buf[0] = (pos >> 24) & 0xFF;
    msg.buf[1] = (pos >> 16) & 0xFF;
    msg.buf[2] = (pos >> 8) & 0xFF;
    msg.buf[3] = pos & 0xFF;

    Can3.write(msg);
  }
}

void printAllROMs() {
  Serial.println();
  Serial.println("==== Motor ROM Summary ====");
  for (int i = 0; i < 6; i++) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" | LowROM: ");
    Serial.print(lowROM[i]);
    Serial.print(" | HighROM: ");
    Serial.print(highROM[i]);
    Serial.print(" | Range: ");
    Serial.println(highROM[i] - lowROM[i]);
  }
  Serial.println("============================");
}
