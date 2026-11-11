/*
  Rubik's Cube Collector Robot - GRID version
  (Arduino Nano + MNBOT Motor driver v1.1, 2 DC motors, 5-ch IR line array)

  Grid model:
    - Start/drop cell is (0,0). x grows to the robot's right (East), y grows forward (North)
      relative to the way the robot faces at start (START_HEADING).
    - The robot counts intersections as it drives, so it always knows its (x,y) and heading.
    - The cube is carried BETWEEN THE FIXED ARMS (the yellow U-shaped body). No gripper moves.
    - For each of the 3 cubes in CUBES[]: drive into its cell so the cube slides into the U,
      turn around gently, drive to the DROP cell, then back up so the cube stays there.
      After all cubes are done, every cube is gathered in the one DROP cell.

  Place the robot ON the start intersection (0,0), facing along a grid line.
  Set TEST_MODE = true first to check wiring (sensors print on Serial; motors/servos wiggle).
  !!! CHECK THE PIN BLOCK AGAINST YOUR WIRING BEFORE UPLOADING !!!
*/

#include <Servo.h>

// ================= PIN CONFIG (edit to match your wiring) =================
const uint8_t PIN_ML_A = 5;   // left motor  pin A (PWM)
const uint8_t PIN_ML_B = 6;   // left motor  pin B (PWM)
const uint8_t PIN_MR_A = 9;   // right motor pin A (PWM)
const uint8_t PIN_MR_B = 10;  // right motor pin B (PWM)
const uint8_t PIN_SERVO_L = 2;   // only used if USE_SERVOS = true
const uint8_t PIN_SERVO_R = 3;
const uint8_t SENSOR_PINS[5] = {A0, A1, A2, A3, A4};   // OUT1..OUT5, left -> right
// ===========================================================================

// ================= GRID / MISSION CONFIG =================
// Cube cells {x, y}. (0,0) is the start/drop intersection.
const int8_t CUBES[][2] = { {1, 2}, {2, 1}, {0, 3} };
const int NUM_CUBES = sizeof(CUBES) / sizeof(CUBES[0]);
// Where ALL cubes get gathered. Default = the start cell. Any cell works.
const int8_t DROP_X = 0, DROP_Y = 0;
const bool X_FIRST = true;      // move along x before y
// Grid size is NOT needed: the robot only counts intersections to each cube.
// To fill CUBES: count intersections from the start. 1 right + 2 forward => {1, 2}.
// ==========================================================

// ---- Tuning ----
const bool  TEST_MODE        = false;
const bool  USE_SERVOS       = false;   // arms are fixed; leave false (servos stay unused)
const bool  LINE_IS_BLACK    = true;    // flip if the robot runs away from the line
const int   BASE_SPEED       = 140;
const int   TURN_FAST        = 140;     // turns: outer motor speed
const int   TURN_SLOW        = 20;      // turns: inner motor speed (0 = pivot on one wheel)
const float KP = 28.0, KD = 18.0;
const int   NODE_NUDGE_MS    = 220;     // roll forward so the wheels sit on the intersection
const int   TURN_MIN_MS      = 450;     // arc time before looking for the new line (90 deg) - tune
const int   UTURN_MIN_MS     = 600;     // in-place spin time for 180 deg - tune
const int   UTURN_SPEED      = 130;     // in-place spin speed (lowered automatically while carrying)
const int   NODE_DEBOUNCE_MS = 300;     // ignore intersections right after leaving one
const int   GRAB_NUDGE_MS    = 200;     // extra creep so the cube is fully inside the U
const int   CARRY_TURN_FAST  = 110;     // gentler turns while carrying so the cube stays in
const int   BACKUP_MS        = 450;
const int   DEADEND_MS       = 400;     // no line this long = dead end -> U-turn back

const int L_OPEN = 20,  L_CLOSED = 80;    // servo angles - tune on the robot
const int R_OPEN = 160, R_CLOSED = 100;   // mirrored

Servo servoL, servoR;

// Heading: 0=North(+y) 1=East(+x) 2=South(-y) 3=West(-x)
const int START_HEADING = 0;
int heading = START_HEADING;
int posX = 0, posY = 0;
float lastError = 0;

// ---------------- Motors ----------------
void setMotor(uint8_t a, uint8_t b, int s) {
  s = constrain(s, -255, 255);
  if (s >= 0) { analogWrite(a, s); analogWrite(b, 0); }
  else        { analogWrite(a, 0); analogWrite(b, -s); }
}
void drive(int l, int r) { setMotor(PIN_ML_A, PIN_ML_B, l); setMotor(PIN_MR_A, PIN_MR_B, r); }
void stopMotors() { drive(0, 0); }

// ---------------- Grippers ----------------
void openGrip()  { if (USE_SERVOS) { servoL.write(L_OPEN);   servoR.write(R_OPEN);   delay(500); } }
void closeGrip() { if (USE_SERVOS) { servoL.write(L_CLOSED); servoR.write(R_CLOSED); delay(600); } }

// ---------------- Sensors ----------------
bool readSensor(int i) {
  bool v = digitalRead(SENSOR_PINS[i]);
  return LINE_IS_BLACK ? v : !v;      // true = line seen
}
int countOn() { int n = 0; for (int i = 0; i < 5; i++) if (readSensor(i)) n++; return n; }

bool linePosition(float &pos) {       // -4 (far left) .. +4 (far right)
  static const int w[5] = {-4, -2, 0, 2, 4};
  int sum = 0, n = 0;
  for (int i = 0; i < 5; i++) if (readSensor(i)) { sum += w[i]; n++; }
  if (n == 0) return false;
  pos = (float)sum / n;
  return true;
}

// Intersection: 4+ sensors, or a T/corner (centre + all sensors on one side)
bool atIntersection() {
  if (countOn() >= 4) return true;
  bool s0 = readSensor(0), s1 = readSensor(1), s2 = readSensor(2),
       s3 = readSensor(3), s4 = readSensor(4);
  return (s0 && s1 && s2) || (s2 && s3 && s4);
}

void followStep() {
  float pos;
  if (!linePosition(pos)) {           // lost line: turn toward where we last saw it
    if (lastError < 0) drive(TURN_SLOW, TURN_FAST);
    else               drive(TURN_FAST, TURN_SLOW);
    return;
  }
  float c = KP * pos + KD * (pos - lastError);
  lastError = pos;
  drive(BASE_SPEED + c, BASE_SPEED - c);
}

// ---------------- Turning ----------------
bool carrying = false;
// Turn WITHOUT rotating in place: one motor slower, the other faster (both go forward).
void spin(int dir, int minMs) {       // dir: -1 left, +1 right
  int fast = carrying ? CARRY_TURN_FAST : TURN_FAST;
  if (dir < 0) drive(TURN_SLOW, fast);   // left turn: left motor slower
  else         drive(fast, TURN_SLOW);   // right turn: right motor slower
  delay(minMs);
  unsigned long t = millis();
  while (!readSensor(2) && millis() - t < 2500) {}
  stopMotors(); lastError = 0; delay(100);
}

// U-turn: rotate IN PLACE (one wheel forward, the other backward).
void uTurn(int minMs) {
  int sp = carrying ? CARRY_TURN_FAST : UTURN_SPEED;
  drive(sp, -sp);
  delay(minMs);
  unsigned long t = millis();
  while (!readSensor(2) && millis() - t < 2500) {}   // until the centre sensor finds the line
  stopMotors(); lastError = 0; delay(100);
}

void turnTo(int want) {
  int diff = (want - heading + 4) % 4;      // 0 straight, 1 right, 2 back, 3 left
  if (diff == 1)      spin(+1, TURN_MIN_MS);
  else if (diff == 3) spin(-1, TURN_MIN_MS);
  else if (diff == 2) uTurn(UTURN_MIN_MS);
  heading = want;
}

// ---------------- Grid movement ----------------
// Follow the line until the next intersection, center on it, update position.
void driveToNextNode() {
  unsigned long start = millis(), lostSince = 0;
  while (true) {
    if (millis() - start > NODE_DEBOUNCE_MS && atIntersection()) break;
    if (countOn() == 0) {                       // no line under the robot
      if (!lostSince) lostSince = millis();
      if (millis() - lostSince > DEADEND_MS) {  // dead end: U-turn in place, go back
        uTurn(UTURN_MIN_MS);
        heading = (heading + 2) % 4;
        lostSince = 0;
        start = millis();
        // now heading back to the node we left: no position change when we reach it
        while (!(millis() - start > NODE_DEBOUNCE_MS && atIntersection())) followStep();
        drive(BASE_SPEED, BASE_SPEED); delay(NODE_NUDGE_MS); stopMotors();
        return;                                 // navigateTo() will plan again
      }
    } else lostSince = 0;
    followStep();
  }
  drive(BASE_SPEED, BASE_SPEED); delay(NODE_NUDGE_MS); stopMotors();
  if (heading == 0) posY++;
  else if (heading == 1) posX++;
  else if (heading == 2) posY--;
  else posX--;
}

void navigateTo(int tx, int ty) {
  while (posX != tx || posY != ty) {
    bool moveX = (posX != tx) && (X_FIRST || posY == ty);
    if (!moveX && posY == ty) moveX = true;
    int want = moveX ? (tx > posX ? 1 : 3) : (ty > posY ? 0 : 2);
    turnTo(want);
    driveToNextNode();
  }
}

// ---------------- Test mode ----------------
void testLoop() {
  for (int i = 0; i < 5; i++) Serial.print(readSensor(i) ? '#' : '.');
  Serial.println(atIntersection() ? "  <- intersection" : "");
  static unsigned long t = 0;
  if (millis() - t > 3000) {
    t = millis();
    drive(150, 0); delay(500); stopMotors();
    drive(0, 150); delay(500); stopMotors();
    openGrip(); closeGrip(); openGrip();
  }
  delay(100);
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);
  pinMode(PIN_ML_A, OUTPUT); pinMode(PIN_ML_B, OUTPUT);
  pinMode(PIN_MR_A, OUTPUT); pinMode(PIN_MR_B, OUTPUT);
  if (USE_SERVOS) { servoL.attach(PIN_SERVO_L); servoR.attach(PIN_SERVO_R); }
  openGrip();
  stopMotors();
  delay(2000);                        // place the robot on the start intersection
}

void loop() {
  if (TEST_MODE) { testLoop(); return; }

  for (int i = 0; i < NUM_CUBES; i++) {
    navigateTo(CUBES[i][0], CUBES[i][1]);
    drive(BASE_SPEED, BASE_SPEED); delay(GRAB_NUDGE_MS); stopMotors();
    carrying = true;                               // cube is now inside the U

    navigateTo(DROP_X, DROP_Y);
    carrying = false;                              // reversing out leaves the cube behind
    drive(-BASE_SPEED, -BASE_SPEED); delay(BACKUP_MS); stopMotors();   // cube stays in the drop cell
  }
  turnTo(START_HEADING);
  while (true) stopMotors();                       // done
}
