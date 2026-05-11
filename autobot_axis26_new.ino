//  AUTOBOT — AXIS'26 VNIT NAGPUR
#include <Arduino.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// ================================================================
//  SENSOR PINS
// ================================================================
#define SENSOR_COUNT 8
const uint8_t qtrPins[SENSOR_COUNT] = {36, 39, 34, 35, 32, 33, 25, 26};

// ================================================================
//  MOTOR PINS
// ================================================================
#define AIN1 18
#define AIN2 5
#define PWMA 4

#define BIN1 19
#define BIN2 21
#define PWMB 22

#define STBY -1

Motor motorA = Motor(AIN1, AIN2, PWMA, -1, STBY);
Motor motorB = Motor(BIN1, BIN2, PWMB, -1, STBY);

// ================================================================
//  BUTTON & LED PINS
// ================================================================
#define LED_LEFT  13
#define LED_RIGHT 14

#define BTN1 12
#define BTN2 27

// ================================================================
//  MOTOR TRIM
//  Tune these two values only. One should always be 0.
//  Bot drifts RIGHT → increase RIGHT_TRIM (reduces right motor)
//  Bot drifts LEFT  → increase LEFT_TRIM  (reduces left motor)
// ================================================================
#define LEFT_TRIM   0
#define RIGHT_TRIM  10   // try -20, then -30, then -40  // <-- your confirmed working value

// ================================================================
//  SPEEDS
// ================================================================
#define BASE_SPEED  140
#define TURN_SPEED  120
#define TURN_SLOW   60
#define CALIB_SPEED 80

// ================================================================
//  PID CONSTANTS
// =============================================================
float Kp = 0.015;//0.011
float Ki = 0.0001;
float Kd = 0.12;//0.11

float lastError = 0;
float integral  = 0;

// ================================================================
//  SENSOR STATE
// ================================================================
QTRSensors qtr;
uint16_t   sensorValues[SENSOR_COUNT];
uint16_t   rawValues[SENSOR_COUNT];
uint16_t   rawMin[SENSOR_COUNT];
uint16_t   rawMax[SENSOR_COUNT];
uint16_t   thresholds[SENSOR_COUNT];

// ================================================================
//  MODE DEFINITIONS
// ================================================================
#define MODE_LINE_FOLLOW        0
#define MODE_LINE_FOLLOW_LEFT   1
#define MODE_LINE_FOLLOW_RIGHT  2

int selectedMode = 0;

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
void  calibrateSensors();
void  readSensors();
int   countActive();
bool  leftActive();
bool  rightActive();
bool  deadCenter();
bool  centerActive();
bool  intersection();
bool  isJunction();
bool  isDeadEnd();

void  followLinePID();
void  resetPID();

void  handleJunction();
void  handleJunctionLineFollow();
void  handleDeadEnd();
void  handleDeadEndLineFollow();
char  decideJunction(bool useLSRB);
void  optimizePath();
void  executeDecision(char decision);

void  turnLeft();
void  turnRight();
void  turnBack();

void  setLeft(int s);
void  setRight(int s);
void  stopMotors();
void  setLED(char direction);
void  clearLEDs();
void  showModeIndicator(int mode);

// ================================================================
//  SETUP
// ================================================================
void setup() {
  pinMode(BTN1,      INPUT_PULLUP);
  pinMode(BTN2,      INPUT_PULLUP);
  pinMode(LED_LEFT,  OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  digitalWrite(LED_LEFT,  HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  delay(500);
  digitalWrite(LED_LEFT,  LOW);
  digitalWrite(LED_RIGHT, LOW);

  qtr.setTypeAnalog();
  qtr.setSensorPins(qtrPins, SENSOR_COUNT);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    rawMin[i] = 4095;
    rawMax[i] = 0;
  }

  while (true) {
    if (digitalRead(BTN2) == LOW) {
      calibrateSensors();
      selectedMode = 0;
      break;
    }
  }

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_LEFT,  HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    delay(150);
    digitalWrite(LED_LEFT,  LOW);
    digitalWrite(LED_RIGHT, LOW);
    delay(150);
  }

  showModeIndicator(selectedMode);

  while (true) {
    if (digitalRead(BTN1) == LOW) {
      selectedMode = (selectedMode + 1) % 3;
      showModeIndicator(selectedMode);
      delay(350);
    }
    if (digitalRead(BTN2) == LOW) {
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_LEFT,  HIGH);
        digitalWrite(LED_RIGHT, HIGH);
        delay(200);
        digitalWrite(LED_LEFT,  LOW);
        digitalWrite(LED_RIGHT, LOW);
        delay(200);
      }
      delay(300);
      break;
    }
  }

  clearLEDs();
  resetPID();
}

// ================================================================
//  MODE INDICATOR
// ================================================================
void showModeIndicator(int mode) {
  clearLEDs();
  delay(100);
  switch (mode) {
    case MODE_LINE_FOLLOW:
      digitalWrite(LED_LEFT,  HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      break;
    case MODE_LINE_FOLLOW_LEFT:
      digitalWrite(LED_LEFT, HIGH);
      break;
    case MODE_LINE_FOLLOW_RIGHT:
      digitalWrite(LED_RIGHT, HIGH);
      break;
  }
}

// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {
  readSensors();

  if (isDeadEnd()) {
    handleDeadEndLineFollow();
    return;
  }

  if (isJunction()) {
    handleJunctionLineFollow();
    return;
  }

  followLinePID();
}

bool isJunction() {
  return ((leftActive() || rightActive()) && (centerActive()));
}

bool isDeadEnd() { return countActive() == 0; }

// ================================================================
//  SENSOR REGION HELPERS
// ================================================================
bool rightActive()  { return rawValues[0] > thresholds[0] && rawValues[1] > thresholds[1]; }
bool leftActive()   { return rawValues[6] > thresholds[6] && rawValues[7] > thresholds[7]; }
bool centerActive() { return rawValues[3] > thresholds[3] && rawValues[4] > thresholds[4]; }
bool deadCenter()   { return rawValues[2] > thresholds[2] || rawValues[3] > thresholds[3] || rawValues[4] > thresholds[4] || rawValues[5] > thresholds[5]; }

// ================================================================
//  HANDLE JUNCTION
// ================================================================
void handleJunctionLineFollow() {
  int  priority = selectedMode;
  bool isLeft   = leftActive();
  bool isRight  = rightActive();

  setLeft(BASE_SPEED);
  setRight(BASE_SPEED);
  delay(100);

  stopMotors();
  readSensors();

  if (priority == 0) {
    if (centerActive()) {
      resetPID();
      return;
    } 
    else if (isLeft) {
      turnLeft();
    } 
    else {
      turnRight();
    }
  }
  else if (priority == 1) {
    if (isLeft) {
      turnLeft();
    } 
    else if (centerActive()) {
      resetPID();
      return;
    } 
    else {
      turnRight();
    }
  }
  else {
    if (isRight) {
      turnRight();
    } 
    else if (centerActive()) {
      resetPID();
      return;
    } 
    else {
      turnLeft();
    }
  }

  resetPID();
}

// ================================================================
//  HANDLE DEAD END
// ================================================================
void handleDeadEndLineFollow() {
  unsigned long t = millis() + 1000;
  int  priority = selectedMode;
  while (!deadCenter() && millis() < t) {
    qtr.read(rawValues);
    if(priority == 0){
      setLeft(TURN_SPEED);
      setRight(-TURN_SLOW);
    }
    else if(priority == 1){
      setLeft(-TURN_SLOW);
      setRight(TURN_SPEED);
    }
    else{
      setLeft(TURN_SPEED);
      setRight(-TURN_SLOW);
    }
  }
  resetPID();
}

// ================================================================
//  TURN LEFT
// ================================================================
void turnLeft() {
  digitalWrite(LED_LEFT,HIGH);
  if (centerActive()) {
    unsigned long t = millis() + 1500;
    while (centerActive() && millis() < t) {
      qtr.read(rawValues);
      setLeft(-TURN_SLOW);
      setRight(TURN_SPEED);
    }
  }
  unsigned long t = millis() + 2000;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    setLeft(-TURN_SLOW);
    setRight(TURN_SPEED);
  }
  stopMotors();
  delay(50);
  digitalWrite(LED_LEFT,LOW);
}

// ================================================================
//  TURN RIGHT
// ================================================================
void turnRight() {
  digitalWrite(LED_RIGHT,HIGH);
  if (centerActive()) {
    unsigned long t = millis() + 1500;
    while (centerActive() && millis() < t) {
      qtr.read(rawValues);
      setLeft(TURN_SPEED);
      setRight(-TURN_SLOW);
    }
  }
  unsigned long t = millis() + 2000;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    setLeft(TURN_SPEED);
    setRight(-TURN_SLOW);
  }
  stopMotors();
  delay(50);
  digitalWrite(LED_RIGHT,LOW);
}

// ================================================================
//  PID LINE FOLLOWING
// ================================================================
void followLinePID() {
  int position = qtr.readLineBlack(sensorValues);
  int error    = 3500 - position;

  integral += error;
  integral   = constrain(integral, -5000, 5000);   // anti-windup

  float derivative = error - lastError;
  float output     = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Trim applied here too so PID straight-running is also balanced
  int leftSpeed  = constrain(BASE_SPEED + (int)output - LEFT_TRIM,  -255, 255);
  int rightSpeed = constrain(BASE_SPEED - (int)output + RIGHT_TRIM, -255, 255);

  setLeft(leftSpeed);
  setRight(rightSpeed);
}

void resetPID() {
  lastError = 0;
  integral  = 0;
}

// ================================================================
//  SENSOR HELPERS
// ================================================================
void calibrateSensors() {
  for (int i = 0; i < 400; i++) {
    setLeft(CALIB_SPEED);
    setRight(-CALIB_SPEED);

    qtr.calibrate();
    uint16_t temp[SENSOR_COUNT];
    qtr.read(temp);
    for (int j = 0; j < SENSOR_COUNT; j++) {
      if (temp[j] < rawMin[j]) rawMin[j] = temp[j];
      if (temp[j] > rawMax[j]) rawMax[j] = temp[j];
    }
    delay(10);
  }

  stopMotors();

  for (int i = 0; i < SENSOR_COUNT; i++)
    thresholds[i] = (rawMin[i] + rawMax[i]) / 2;
}

void readSensors() {
  qtr.read(rawValues);
  qtr.readCalibrated(sensorValues);
}

int countActive() {
  int c = 0;
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (rawValues[i] > thresholds[i]) c++;
  return c;
}

// ================================================================
//  LED HELPERS
// ================================================================
void clearLEDs() {
  digitalWrite(LED_LEFT,  LOW);
  digitalWrite(LED_RIGHT, LOW);
}

// ================================================================
//  MOTOR HELPERS  — trim baked in here
// ================================================================
void setLeft(int s)  { motorA.drive(constrain(s - LEFT_TRIM,  -255, 255)); }
void setRight(int s) { motorB.drive(constrain(s + RIGHT_TRIM, -255, 255)); }
void stopMotors()    { motorA.brake(); motorB.brake(); }