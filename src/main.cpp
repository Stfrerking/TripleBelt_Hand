// Version: 2025-04-18
// Description: Initial PlatformIO version — Motor 0 homing + joystick drive + servo configs


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

// ----------- Joystick and Button ----------- //
#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define BUTTON_PIN 33
#define DEBOUNCE_DELAY 50

volatile bool configChangeRequested = false;

// Global ROM limits for Motor Limits
long lowROM[6]  = {-100000, -100000, -100000, -100000, -100000, -100000};
long highROM[6] = {100000, 100000, 100000, 100000, 100000, 100000};

// ----------- Homing Sequence ----------- //

#define HOMING_TIMEOUT 8000   // Max time allowed (ms)

Encoder* encoders[6] = {
  &encoder0, &encoder1, &encoder2,
  &encoder3, &encoder4, &encoder5
};

const int motorPWM[6]   = {MOTOR0_PWM, MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM, MOTOR5_PWM};
const int motorDIR_A[6] = {MOTOR0_DIR_A, MOTOR1_DIR_A, MOTOR2_DIR_A, MOTOR3_DIR_A, MOTOR4_DIR_A, MOTOR5_DIR_A};
const int motorDIR_B[6] = {MOTOR0_DIR_B, MOTOR1_DIR_B, MOTOR2_DIR_B, MOTOR3_DIR_B, MOTOR4_DIR_B, MOTOR5_DIR_B};
const int motorSEN[6] = {MOTOR0_SEN, MOTOR1_SEN, MOTOR2_SEN, MOTOR3_SEN, MOTOR4_SEN, MOTOR5_SEN};

// Stall current thresholds [A]
float stallThresholds[6] = {.6, .6, .6, .6, .6, .6};

// Homing speeds [0-100] PWM will be scalled later
const int homingSpeeds[6] = {80, 80, 80, 80, 80, 80};

// Backoff durations [ms]
const int backoffTimes[6] = {50, 100, 50, 100, 50, 100};

// Pullback distances after stall [encoder counts]
const int pullbackCounts[6] = {10, 10, 10, 10, 10, 10};

// Stall debounce [ms] — time without motion before we consider it stuck
const unsigned long stallDebounce[6] = {60, 80, 60, 80, 60, 80};

//Joystick Variables
const int DEADZONE = 10;
const int MAX_INPUT = 4095;
const int MID_POINT = 2047;

int processedX = 0;
int processedY = 0;

//Declaration for finger configurations driven by servos
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
void controlMotor(int index, int speed, long lowROM, long highROM);

void homeMotors();

void nextState();
void handleState();
void printEncoderData();
void printAllROMs();

PWMServo servo1;

void configChangeISR() {
  configChangeRequested = true;
}

float lerp(float x, float x1, float x2, float y1, float y2) {
  return y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
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

  homeMotors();
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

  controlMotor(0, processedX, lowROM[0], highROM[0]);
  controlMotor(1, processedY, lowROM[1], highROM[1]);
  controlMotor(2, processedX, lowROM[2], highROM[2]);
  controlMotor(3, processedY, lowROM[3], highROM[3]);
  controlMotor(4, processedX, lowROM[4], highROM[4]);
  controlMotor(5, processedY, lowROM[5], highROM[5]);


  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    Serial.print("X: "); Serial.print(processedX); Serial.print("	Y: "); Serial.println(processedY);
    printEncoderData();
   
    for (int i = 0; i < 6; i++) {
      int raw = analogRead(motorSEN[i]);
      float voltage = raw * (3.3 / 4095.0);
      float current = voltage / 0.5; // update if shunt value changes
    
      Serial.print("M"); Serial.print(i);
      Serial.print(": ADC="); Serial.print(raw);
      Serial.print("  I="); Serial.print(current, 3);
      Serial.print(" A   ");
    }
    Serial.println();
    

    
    
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

void controlMotor(int index, int speed, long lowROM, long highROM) {
  long pos = encoders[index]->read();
  float normalizedPos = (float)(pos - lowROM) / (highROM - lowROM);
  float target = (float)speed / 100.0; // normalize input to -1.0 to 1.0

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
  Serial.print("Enc0: "); Serial.print(encoder0.read()); Serial.print("\t");
  Serial.print("Enc1: "); Serial.print(encoder1.read()); Serial.print("\t");
  Serial.print("Enc2: "); Serial.print(encoder2.read()); Serial.print("\t");
  Serial.print("Enc3: "); Serial.print(encoder3.read()); Serial.print("\t");
  Serial.print("Enc4: "); Serial.print(encoder4.read()); Serial.print("\t");
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

  // -------- Move to low-end stop -------- //
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

    Serial.print("Motor ");
    Serial.print(index);
    Serial.print(" | Pos: ");
    Serial.print(currPos);
    Serial.print(" | Δt: ");
    Serial.print(timeSinceMove);
    Serial.print(" ms | Current: ");
    Serial.print(currentFiltered, 3);
    Serial.print(" A | OverCurrent: ");
    Serial.print(overCurrent);
    Serial.print(" | Stuck: ");
    Serial.println(stuck);

    if (stuck && overCurrent) {
      Serial.println("Low-end stall detected!");
      break;
    }

    delay(20);
  }

  // Stop and back off slightly
  controlMotor(index, 0, lowROM[index], highROM[index]);
  controlMotor(index, 100, -100000, 100000);
  delay(backoffTimes[index]);
  controlMotor(index, 0, lowROM[index], highROM[index]);

  lowROM[index] = encoders[index]->read() + pullbackCounts[index];
  encoders[index]->write(0);
  Serial.print("Set lowROM["); Serial.print(index); Serial.print("]: ");
  Serial.println(lowROM[index]);

  // -------- Move to high-end stop -------- //
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

    Serial.print("Motor ");
    Serial.print(index);
    Serial.print(" | Pos: ");
    Serial.print(currPos);
    Serial.print(" | Δt: ");
    Serial.print(timeSinceMove);
    Serial.print(" ms | Current: ");
    Serial.print(currentFiltered, 3);
    Serial.print(" A | OverCurrent: ");
    Serial.print(overCurrent);
    Serial.print(" | Stuck: ");
    Serial.println(stuck);

    if (stuck && overCurrent) {
      Serial.println("High-end stall detected!");
      break;
    }

    delay(20);
  }

  // Stop and back off slightly
  controlMotor(index, 0, lowROM[index], highROM[index]);
  controlMotor(index, -100, lowROM[index], highROM[index]);
  delay(backoffTimes[index]);
  controlMotor(index, 0, lowROM[index], highROM[index]);

  highROM[index] = encoders[index]->read() - pullbackCounts[index];
  Serial.print("Set highROM["); Serial.print(index); Serial.print("]: ");
  Serial.println(highROM[index]);

  // -------- Move to midpoint -------- //
  long midpoint = (lowROM[index] + highROM[index]) / 2;
  long range = highROM[index] - lowROM[index];
  Serial.print("Calculated midpoint: ");
  Serial.println(midpoint);
  Serial.print("ROM Range: ");
  Serial.println(range);

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

    Serial.print("Moving to midpoint | Pos: ");
    Serial.print(currentPos);
    Serial.print(" | Target: ");
    Serial.print(midpoint);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Speed: ");
    Serial.println(direction * speed);

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
  for (int i = 0; i < 6; i++) {
    homeMotor(i, motorSEN[i]);
  }
  printAllROMs();
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
