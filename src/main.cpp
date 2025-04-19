#include <PWMServo.h>
#include <Arduino.h>
#include <Encoder.h>

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

Encoder encoder0(ENC0_B, ENC0_A);
Encoder encoder1(ENC1_B, ENC1_A);
Encoder encoder2(ENC2_B, ENC2_A);
Encoder encoder3(ENC3_B, ENC3_A);
Encoder encoder4(ENC4_B, ENC4_A);
Encoder encoder5(ENC5_B, ENC5_A);

// ----------- Motor Driver Pins ----------- //
#define MOTOR0_PWM 4
#define MOTOR0_DIR_A 25
#define MOTOR0_DIR_B 26

#define MOTOR1_PWM 5
#define MOTOR1_DIR_A 27
#define MOTOR1_DIR_B 30

#define MOTOR2_PWM 10
#define MOTOR2_DIR_A 31
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

// ----------- Servo Pins ----------- //
#define SERVO0 28
#define SERVO1 29

// ----------- Joystick and Button ----------- //
#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define BUTTON_PIN 33
#define DEBOUNCE_DELAY 50

#define CURRENT0_PIN A2       // ADC pin for Motor 0 current sensing
#define HOMING_SPEED 80       // PWM value during homing (0-255)
#define HOMING_TIMEOUT 8000   // Max time allowed (ms)
#define HOMING_BACKOFF 50     // How long to reverse after hitting stop
#define STALL_CURRENT .5  // Increased sensitivity     // Amps (tune if needed)
#define STALL_DEBOUNCE 60  // Detect stall faster    // ms to confirm it's a stall

volatile bool configChangeRequested = false;

// Global ROM limits for Motor 0
long lowROM_0 = -100000;
long highROM_0 = 100000;

const int DEADZONE = 10;
const int MAX_INPUT = 4095;
const int MID_POINT = 2047;

int processedX = 0;
int processedY = 0;

enum HandState {
  CONFIG_1,
  CONFIG_2,
  CONFIG_3,
  CONFIG_4
};

HandState currentState = CONFIG_1;
bool buttonState = false;
bool lastButtonState = false;
unsigned long lastDebounceTime = 0;

PWMServo servo0;

// Forward declarations
void controlMotor0(int speed, long lowROM = 0, long highROM = 100000);
void controlMotor1(int speed);
void controlMotor2(int speed);
void controlMotor3(int speed);
void controlMotor4(int speed);
void controlMotor5(int speed);

void nextState();
void handleState();
void printEncoderData();

PWMServo servo1;

void configChangeISR() {
  configChangeRequested = true;
}

float lerp(float x, float x1, float x2, float y1, float y2) {
  return y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
}

void homeMotor0() {
  Serial.println("Starting homing sequence for Motor 0");
  long prevPos = encoder0.read();
  unsigned long lastMoveTime = millis();
  unsigned long startTime = millis();
  float alpha = 0.2;
  static float currentFiltered = 0;

  // Move to low end
  controlMotor0(-HOMING_SPEED, -100000, 100000); // override bounds during homing

  while (millis() - startTime < HOMING_TIMEOUT) {
    long currPos = encoder0.read();
    bool encoderMoved = (currPos != prevPos);
    unsigned long now = millis();
    unsigned long timeSinceMove = now - lastMoveTime;

    if (encoderMoved) {
      lastMoveTime = now;
      prevPos = currPos;
    }

    int rawADC = analogRead(CURRENT0_PIN);
    float voltage = rawADC * (3.3 / 4095.0);
    float current = voltage / 0.5;
    currentFiltered = alpha * current + (1 - alpha) * currentFiltered;

    bool stuck = timeSinceMove > STALL_DEBOUNCE;
    bool overCurrent = currentFiltered > STALL_CURRENT;

    Serial.print("EncPos: "); Serial.print(currPos);
    Serial.print(" | Δt: "); Serial.print(timeSinceMove);
    Serial.print(" ms | Moved: "); Serial.print(encoderMoved);
    Serial.print(" | Current: "); Serial.print(currentFiltered, 3);
    Serial.print(" A | Stuck: "); Serial.print(stuck);
    Serial.print(" | OverCurrent: "); Serial.print(overCurrent);
    Serial.print(" => Stall? "); Serial.println(stuck && overCurrent);

    if (stuck && overCurrent) {
      Serial.println("Low-end stall detected!");
      break;
    }

    delay(20);
  }

  controlMotor0(0, lowROM_0, highROM_0);
  controlMotor0(HOMING_SPEED / 2, -100000, 100000);
  delay(HOMING_BACKOFF);
  controlMotor0(0);
  lowROM_0 = encoder0.read() + 9000;  // Pull back from hard stop
  encoder0.write(0);
  Serial.println("Low-end homing complete. Encoder 0 set to 0.");

  // Move to high end
  startTime = millis();
  prevPos = encoder0.read();
  lastMoveTime = millis();
  controlMotor0(HOMING_SPEED, -100000, 100000);

  while (millis() - startTime < HOMING_TIMEOUT) {
    long currPos = encoder0.read();
    bool encoderMoved = (currPos != prevPos);
    unsigned long now = millis();
    unsigned long timeSinceMove = now - lastMoveTime;

    if (encoderMoved) {
      lastMoveTime = now;
      prevPos = currPos;
    }

    int rawADC = analogRead(CURRENT0_PIN);
    float voltage = rawADC * (3.3 / 4095.0);
    float current = voltage / 0.5;
    currentFiltered = alpha * current + (1 - alpha) * currentFiltered;

    bool stuck = timeSinceMove > STALL_DEBOUNCE;
    bool overCurrent = currentFiltered > STALL_CURRENT;

    Serial.print("EncPos: "); Serial.print(currPos);
    Serial.print(" | Δt: "); Serial.print(timeSinceMove);
    Serial.print(" ms | Moved: "); Serial.print(encoderMoved);
    Serial.print(" | Current: "); Serial.print(currentFiltered, 3);
    Serial.print(" A | Stuck: "); Serial.print(stuck);
    Serial.print(" | OverCurrent: "); Serial.print(overCurrent);
    Serial.print(" => Stall? "); Serial.println(stuck && overCurrent);

    if (stuck && overCurrent) {
      Serial.println("High-end stall detected!");
      break;
    }

    delay(20);
  }

  controlMotor0(0);
  controlMotor0(-HOMING_SPEED / 2);
  delay(HOMING_BACKOFF);
  controlMotor0(0);
  highROM_0 = encoder0.read() - 1500;  // Pull back from hard stop
  Serial.print("High-end homing complete. Encoder 0 max position: ");
  Serial.println(highROM_0);

  long midpoint = highROM_0 / 2;
  Serial.print("Moving to midpoint: "); Serial.println(midpoint);
  unsigned long moveStart = millis();
  long lastPos = encoder0.read();

  while (abs(encoder0.read() - midpoint) > 10 && millis() - moveStart < HOMING_TIMEOUT) {
    long currentPos = encoder0.read();
    int error = midpoint - currentPos;
    int speed = constrain(map(abs(error), 0, highROM_0 / 2, 80, HOMING_SPEED), 70, HOMING_SPEED);
    int direction = (error > 0) ? 1 : -1;
    controlMotor0(direction * speed);

    Serial.print("Moving toward midpoint | CurrentPos: ");
    Serial.print(currentPos);
    Serial.print(" | Target: ");
    Serial.print(midpoint);
    Serial.print(" | Speed: ");
    Serial.println(direction * speed);

    delay(20);

    if (abs(currentPos - lastPos) < 3) {
      Serial.println("Minimal encoder movement detected, possible stall or dead zone. Holding for retry...");
      delay(200); // Give motor time to overcome inertia or deadband
      long retryPos = encoder0.read();
      if (abs(retryPos - currentPos) < 3) {
        Serial.println("Still no movement after delay. Breaking.");
        break;
      }
    }
    lastPos = currentPos;
  }

  controlMotor0(0);
  Serial.println("Midpoint reached.");
}

void setup() {
  analogReadResolution(12);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), configChangeISR, FALLING);

  int motorPins[] = {
    MOTOR0_PWM, MOTOR0_DIR_A, MOTOR0_DIR_B, MOTOR1_PWM, MOTOR1_DIR_A, MOTOR1_DIR_B,
    MOTOR2_PWM, MOTOR2_DIR_A, MOTOR2_DIR_B, MOTOR3_PWM, MOTOR3_DIR_A, MOTOR3_DIR_B,
    MOTOR4_PWM, MOTOR4_DIR_A, MOTOR4_DIR_B, MOTOR5_PWM, MOTOR5_DIR_A, MOTOR5_DIR_B
  };
  for (size_t i = 0; i < sizeof(motorPins)/sizeof(motorPins[0]); i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  servo0.attach(SERVO0);
  servo1.attach(SERVO1);

  Serial.println("System Initialized");

  homeMotor0();
  handleState();
}

void loop() {

  if (configChangeRequested) {
    configChangeRequested = false;
    nextState();  
  }

  int rawX = analogRead(JOYSTICK_X);
  int rawY = analogRead(JOYSTICK_Y);
  processedX = (int)lerp((float)rawX, 0.0, MAX_INPUT, -100.0, 100.0);
  processedY = (int)lerp((float)rawY, 0.0, MAX_INPUT, -100.0, 100.0);

  processedX = abs(processedX) > DEADZONE ? processedX : 0;
  processedY = abs(processedY) > DEADZONE ? processedY : 0;

  controlMotor0(processedX, lowROM_0, highROM_0);
  controlMotor1(processedY);
  controlMotor2(processedX);
  controlMotor3(processedY);
  controlMotor4(processedX);
  controlMotor5(processedY);

   int raw = analogRead(A2);  // Change A2 to your ADC pin
  float voltage = raw * (3.3 / 1023.0);  // Use 4095.0 if Teensy is in 12-bit mode
  float current = voltage / 0.5;  // Adjust if you used a different sense resistor



  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    Serial.print("X: "); Serial.print(processedX); Serial.print("	Y: "); Serial.println(processedY);
    printEncoderData();
   
    Serial.print("ADC Raw: "); Serial.print(raw);
    Serial.print("  Voltage: "); Serial.print(voltage, 3);
    Serial.print(" V  Current: "); Serial.print(current, 3);
    Serial.println(" A");
    
    lastPrint = millis();
  }
}

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
      servo0.write(180);
      servo1.write(0);
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
      servo0.write(0);
      servo1.write(180);
      break;
  }
}

void controlMotor0(int speed, long lowROM, long highROM) {
  long pos = encoder0.read();
  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0; // normalize input to -1.0 to 1.0

  if ((target > 0 && normalizedPos >= 1.0) || (target < 0 && normalizedPos <= 0.0)) {
    analogWrite(MOTOR0_PWM, 0);
    return;
  }
  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  digitalWrite(MOTOR0_DIR_A, speed < 0);
  digitalWrite(MOTOR0_DIR_B, speed > 0);
  analogWrite(MOTOR0_PWM, pwmValue);
}

void controlMotor1(int speed) {
  long pos = encoder1.read();
  long lowROM = -100000;
  long highROM = 100000;

  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0;

  if ((target > 0 && normalizedPos >= 1.0) || (target < 0 && normalizedPos <= 0.0)) {
    analogWrite(MOTOR1_PWM, 0);
    return;
  }

  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  digitalWrite(MOTOR1_DIR_A, speed < 0);
  digitalWrite(MOTOR1_DIR_B, speed > 0);
  analogWrite(MOTOR1_PWM, pwmValue);
}

void controlMotor2(int speed) {
  long pos = encoder2.read();
  long lowROM = -100000;  // extreme safe range for tuning later
  long highROM = 100000;

  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0;

  if ((target > 0 && normalizedPos >= 1.0) || (target < 0 && normalizedPos <= 0.0)) {
    analogWrite(MOTOR2_PWM, 0);
    return;
  }

  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  digitalWrite(MOTOR2_DIR_A, speed < 0);
  digitalWrite(MOTOR2_DIR_B, speed > 0);
  analogWrite(MOTOR2_PWM, pwmValue);
}
void controlMotor3(int speed) {
  long pos = encoder3.read();
  long lowROM = -100000;
  long highROM = 100000;

  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0;

  if ((target > 0 && normalizedPos >= 1.0) || (target < 0 && normalizedPos <= 0.0)) {
    analogWrite(MOTOR3_PWM, 0);
    return;
  }

  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  digitalWrite(MOTOR3_DIR_A, speed < 0);
  digitalWrite(MOTOR3_DIR_B, speed > 0);
  analogWrite(MOTOR3_PWM, pwmValue);
}
void controlMotor4(int speed) {
  long pos = encoder4.read();
  long lowROM = -100000;
  long highROM = 100000;

  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0;

  if ((target > 0 && normalizedPos >= 1.0) || (target < 0 && normalizedPos <= 0.0)) {
    analogWrite(MOTOR4_PWM, 0);
    return;
  }

  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  digitalWrite(MOTOR4_DIR_A, speed < 0);
  digitalWrite(MOTOR4_DIR_B, speed > 0);
  analogWrite(MOTOR4_PWM, pwmValue);
}
void controlMotor5(int speed) {
  long pos = encoder5.read();
  long lowROM = -100000;
  long highROM = 100000;

  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0;

  if ((target > 0 && normalizedPos >= 1.0) || (target < 0 && normalizedPos <= 0.0)) {
    analogWrite(MOTOR5_PWM, 0);
    return;
  }

  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  digitalWrite(MOTOR5_DIR_A, speed < 0);
  digitalWrite(MOTOR5_DIR_B, speed > 0);
  analogWrite(MOTOR5_PWM, pwmValue);
}

void printEncoderData() {
  Serial.print("Enc0: "); Serial.print(encoder0.read()); Serial.print("\t");
  Serial.print("Enc1: "); Serial.print(encoder1.read()); Serial.print("\t");
  Serial.print("Enc2: "); Serial.print(encoder2.read()); Serial.print("\t");
  Serial.print("Enc3: "); Serial.print(encoder3.read()); Serial.print("\t");
  Serial.print("Enc4: "); Serial.print(encoder4.read()); Serial.print("\t");
  Serial.print("Enc5: "); Serial.println(encoder5.read());
}
