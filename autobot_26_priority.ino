// ================================================================
//  AUTOBOT — AXIS'26 VNIT NAGPUR
//
//  BTN1 = Cycle through modes (press repeatedly to select)
//  BTN2 = Confirm and START selected mode
//
//  MODES (cycle with BTN1):
//    0 - LINE FOLLOW ONLY  (Round 1)
//    1 - LSRB DRY RUN      (Round 2 explore, left priority)
//    2 - LSRB FINAL RUN    (Round 2 replay, left priority)
//    3 - RSLB DRY RUN      (Round 2 explore, right priority)
//    4 - RSLB FINAL RUN    (Round 2 replay, right priority)
//
//  LEDs:
//    LEFT LED  = glows when turning left
//    RIGHT LED = glows when turning right
// ================================================================

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
#define LED_LEFT 13
#define LED_RIGHT 14

#define BTN1 12
#define BTN2 27

// ================================================================
//  SPEEDS
// ================================================================
#define BASE_SPEED  120
#define TURN_SPEED  70
#define TURN_SLOW   30
#define BACK_SPEED  70

// ================================================================
//  PID CONSTANTS
//  Tune Kp first (Ki=0, Kd=0), then add Kd to reduce overshoot.
//  Add tiny Ki only if bot drifts consistently to one side.
// ================================================================
float Kp = 0.052;
float Ki = 0.0001;
float Kd = 0.416;

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
#define MODE_LINE_FOLLOW  0
#define MODE_LSRB_DRY    1
#define MODE_LSRB_FINAL  2
#define MODE_RSLB_DRY    3
#define MODE_RSLB_FINAL  4

int selectedMode = 0;

// ================================================================
//  PATH MEMORY
// ================================================================
#define MAX_PATH 50
char path[MAX_PATH];
int  pathLength  = 0;
int  replayIndex = 0;

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
void  calibrateSensors();
void  readSensors();
int   countActive();
bool  leftActive();
bool  rightActive();
bool  centerActive();
bool  isJunction();
bool  isDeadEnd();

void  followLinePID();
void  resetPID();

void  handleJunction();
void  handleDeadEnd();
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

  // Power-on signal
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

  calibrateSensors();

  // Calibration done signal
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_LEFT,  HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    delay(150);
    digitalWrite(LED_LEFT,  LOW);
    digitalWrite(LED_RIGHT, LOW);
    delay(150);
  }

  showModeIndicator(selectedMode);

  // Mode selection loop
  while (true) {
    if (digitalRead(BTN1) == LOW) {
      selectedMode = (selectedMode + 1) % 5;
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
    case MODE_LSRB_DRY:
      digitalWrite(LED_LEFT, HIGH);
      break;
    case MODE_LSRB_FINAL:
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_LEFT, HIGH); delay(200);
        digitalWrite(LED_LEFT, LOW);  delay(200);
      }
      break;
    case MODE_RSLB_DRY:
      digitalWrite(LED_RIGHT, HIGH);
      break;
    case MODE_RSLB_FINAL:
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_RIGHT, HIGH); delay(200);
        digitalWrite(LED_RIGHT, LOW);  delay(200);
      }
      break;
  }
}

// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {
  readSensors();

  if (isDeadEnd()) {
    handleDeadEnd();
    return;
  }

  if (isJunction()) {
    handleJunction();
    return;
  }

  followLinePID();
}

// ================================================================
//  IS JUNCTION?
// ================================================================
bool isJunction() {
  bool le = leftActive();
  bool re = rightActive();
  int  ac = countActive();

  if (le && re)      return true;
  if (le && ac >= 4) return true;
  if (re && ac >= 4) return true;
  return false;
}

bool isDeadEnd() { return countActive() == 0; }

// ================================================================
//  HANDLE JUNCTION
//
//  Step 1 — Nudge forward until junction body clears.
//           "Junction body" = the wide cross-section where paths
//           intersect. We move forward until edge sensors go white,
//           meaning the bot has physically passed the intersection.
//
//  Step 2 — Check if ALL sensors are still black after nudging.
//           If yes → finish line. The wide activation never cleared
//           because it's a solid perpendicular bar, not a junction.
//           If no  → it was a junction. Now classify and turn.
//
//  This single nudge replaces both the old isFinish() probe AND
//  the redundant nudge that was inside turnLeft/turnRight.
//  No duplicate movement anywhere.
// ================================================================
void handleJunction() {

  // Nudge forward until edge sensors clear the junction body
  // Timeout prevents infinite forward if something goes wrong
  unsigned long nudgeTimeout = millis() + 700;
  while ((leftActive() || rightActive()) && millis() < nudgeTimeout) {
    qtr.read(rawValues);
    setLeft(BASE_SPEED);
    setRight(BASE_SPEED);
  }

  // Read fresh after nudge
  qtr.read(rawValues);

  // If all sensors still black → finish line
  if (countActive() >= 6) {
    stopMotors();
    clearLEDs();
    // Blink both LEDs forever to signal completion
    while (true) {
      digitalWrite(LED_LEFT,  HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      delay(400);
      digitalWrite(LED_LEFT,  LOW);
      digitalWrite(LED_RIGHT, LOW);
      delay(400);
    }
  }

  // It was a junction — classify and decide
  bool useLSRB = (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_LSRB_FINAL);
  char decision;

  if (selectedMode == MODE_LSRB_FINAL || selectedMode == MODE_RSLB_FINAL) {
    // Final run: replay stored path
    if (replayIndex < pathLength) {
      decision = path[replayIndex++];
    } else {
      return; // path exhausted, let PID and finish detection handle it
    }
  } else if (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_RSLB_DRY) {
    // Dry run: decide and record
    decision = decideJunction(useLSRB);
    if (pathLength < MAX_PATH) {
      path[pathLength++] = decision;
      optimizePath();
    }
  } else {
    // Line follow mode: always go straight, no recording
    decision = 'S';
  }

  executeDecision(decision);
}

// ================================================================
//  DECIDE JUNCTION
// ================================================================
char decideJunction(bool useLSRB) {
  bool le = leftActive();
  bool re = rightActive();
  bool ce = centerActive();

  if (useLSRB) {
    if (le) return 'L';
    if (ce) return 'S';
    if (re) return 'R';
    return 'U';
  } else {
    if (re) return 'R';
    if (ce) return 'S';
    if (le) return 'L';
    return 'U';
  }
}

// ================================================================
//  EXECUTE DECISION
// ================================================================
void executeDecision(char decision) {
  setLED(decision);
  switch (decision) {
    case 'L': turnLeft();  break;
    case 'R': turnRight(); break;
    case 'S':              break; // already nudged forward, PID takes over
    case 'U': turnBack();  break;
  }
  clearLEDs();
  resetPID();
}

// ================================================================
//  HANDLE DEAD END
// ================================================================
void handleDeadEnd() {
  bool useLSRB = (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_LSRB_FINAL);

  if (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_RSLB_DRY) {
    if (pathLength < MAX_PATH) {
      path[pathLength++] = 'U';
      optimizePath();
    }
  }

  // Reverse until line found
  // setLeft(-BACK_SPEED);
  // setRight(-BACK_SPEED);
  // delay(300);

  // while (countActive() < 2) {
  //   qtr.read(rawValues);
  //   setLeft(-BACK_SPEED);
  //   setRight(-BACK_SPEED);
  // }
  // stopMotors();
  // delay(80);

  // Pivot toward preferred side until center finds line
  while (!centerActive()) {
    qtr.read(rawValues);
    if (useLSRB) { setLeft(-TURN_SLOW); setRight(TURN_SPEED); }
    else          { setLeft(TURN_SPEED); setRight(-TURN_SLOW); }
  }
  stopMotors();
  delay(60);
  resetPID();
}

// ================================================================
//  OPTIMIZE PATH
// ================================================================
void optimizePath() {
  if (pathLength < 3 || path[pathLength - 2] != 'U')
    return;

  int totalAngle = 0;
  for (int m = 1; m <= 3; m++) {
    switch (path[pathLength - m]) {
      case 'R': totalAngle += 90;  break;
      case 'L': totalAngle += 270; break;
      case 'U': totalAngle += 180; break;
      case 'S': totalAngle += 0;   break;
    }
  }
  totalAngle %= 360;

  switch (totalAngle) {
    case 0:   path[pathLength - 3] = 'S'; break;
    case 90:  path[pathLength - 3] = 'R'; break;
    case 180: path[pathLength - 3] = 'U'; break;
    case 270: path[pathLength - 3] = 'L'; break;
  }
  pathLength -= 2;
}

// ================================================================
//  TURN LEFT
//
//  The bot has already nudged past the junction body before
//  this function is called. So no nudge needed here.
//
//  After the nudge, two situations for center sensor:
//
//  CENTER IS BLACK:
//    Bot is still over the main line (common at plus junctions
//    or shallow-angle turns). Must rotate away from it first.
//    → Turn left until center goes WHITE (lost the incoming line)
//    → Then turn left until center goes BLACK (found exit line)
//
//  CENTER IS WHITE:
//    Bot has already cleared the main line during the nudge.
//    Exit line hasn't appeared yet — just keep rotating to find it.
//    → Turn left until center goes BLACK (found exit line)
//
//  Both cases end at the same condition: center on BLACK = exit found.
//  Timeouts on each phase prevent infinite spinning.
// ================================================================
void turnLeft() {
  if (centerActive()) {
    // Center is black — rotate away from incoming line first
    unsigned long t = millis() + 1500;
    while (centerActive() && millis() < t) {
      qtr.read(rawValues);
      setLeft(-TURN_SLOW);
      setRight(TURN_SPEED);
    }
  }

  // Now center is white — rotate until exit line found
  unsigned long t = millis() + 2000;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    setLeft(-TURN_SLOW);
    setRight(TURN_SPEED);
  }

  stopMotors();
  delay(50);
}

// ================================================================
//  TURN RIGHT
//  Mirror of turnLeft — same two-case logic, opposite direction.
// ================================================================
void turnRight() {
  if (centerActive()) {
    // Center is black — rotate away from incoming line first
    unsigned long t = millis() + 1500;
    while (centerActive() && millis() < t) {
      qtr.read(rawValues);
      setLeft(TURN_SPEED);
      setRight(-TURN_SLOW);
    }
  }

  // Now center is white — rotate until exit line found
  unsigned long t = millis() + 2000;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    setLeft(TURN_SPEED);
    setRight(-TURN_SLOW);
  }

  stopMotors();
  delay(50);
}

// ================================================================
//  TURN BACK (U-turn)
//  Reverse to clear, then same two-case pivot logic.
// ================================================================
void turnBack() {
  bool useLSRB = (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_LSRB_FINAL);

  setLeft(-BACK_SPEED);
  setRight(-BACK_SPEED);
  delay(250);
  stopMotors();
  delay(60);

  if (centerActive()) {
    unsigned long t = millis() + 1500;
    while (centerActive() && millis() < t) {
      qtr.read(rawValues);
      if (useLSRB) { setLeft(-TURN_SLOW); setRight(TURN_SPEED); }
      else          { setLeft(TURN_SPEED); setRight(-TURN_SLOW); }
    }
  }

  unsigned long t = millis() + 2500;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    if (useLSRB) { setLeft(-TURN_SLOW); setRight(TURN_SPEED); }
    else          { setLeft(TURN_SPEED); setRight(-TURN_SLOW); }
  }

  stopMotors();
  delay(50);
}

// ================================================================
//  PID LINE FOLLOWING
// ================================================================
void followLinePID() {
  uint32_t weightedSum = 0;
  uint32_t total       = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    weightedSum += (uint32_t)sensorValues[i] * i * 1000;
    total       += sensorValues[i];
  }

  int position = (total > 0) ? (int)(weightedSum / total) : 3500;
  int error    = 3500-position ;

  integral += error;
  integral  = constrain(integral, -5000, 5000);

  float derivative = error - lastError;
  float output     = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed  = constrain(BASE_SPEED + (int)output, -255, 255);
  int rightSpeed = constrain(BASE_SPEED - (int)output, -255, 255);

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
    qtr.calibrate();
    uint16_t temp[SENSOR_COUNT];
    qtr.read(temp);
    for (int j = 0; j < SENSOR_COUNT; j++) {
      if (temp[j] < rawMin[j]) rawMin[j] = temp[j];
      if (temp[j] > rawMax[j]) rawMax[j] = temp[j];
    }
    delay(10);
  }
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

bool rightActive()   { return rawValues[0] > thresholds[0] || rawValues[1] > thresholds[1]; }
bool leftActive()  { return rawValues[6] > thresholds[6] || rawValues[7] > thresholds[7]; }
bool centerActive() { return rawValues[3] > thresholds[3] || rawValues[4] > thresholds[4]; }

// ================================================================
//  LED HELPERS
// ================================================================
void setLED(char direction) {
  clearLEDs();
  if (direction == 'L') digitalWrite(LED_LEFT,  HIGH);
  if (direction == 'R') digitalWrite(LED_RIGHT, HIGH);
}

void clearLEDs() {
  digitalWrite(LED_LEFT,  LOW);
  digitalWrite(LED_RIGHT, LOW);
}

// ================================================================
//  MOTOR HELPERS
// ================================================================
void setLeft(int s)  { motorA.drive(s); }
void setRight(int s) { motorB.drive(s); }
void stopMotors()    { motorA.brake(); motorB.brake(); }
