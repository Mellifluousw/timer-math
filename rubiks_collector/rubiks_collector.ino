/*
  Rubik's Cube Collector Robot  (Arduino Nano + MNBOT Motor driver v1.1)

  Hardware (from photos): 2 DC motors, 2 MG90S gripper servos, 5-channel IR line array.

  How it works:
    - Follows the black line with PD steering.
    - JUNCTION (4-5 sensors black): takes the next turn from the current TRIP string
      ('L','R','S'), and remembers it.
    - DEAD END (line disappears while not carrying): a cube is there -> close grippers,
      U-turn, then replays the remembered turns in reverse back to the start.
    - DROP ZONE: when carrying and the turns are used up (back at the start), the next
      all-black marker opens the grippers, backs up, U-turns and starts the next TRIP.

  Edit TRIPS to match your track: one string per cube, turns in the order you meet junctions.
  Set TEST_MODE = true first to check wiring (sensors print to Serial, motors/servos wiggle).
*/

#include <Servo.h>

// ================= PIN CONFIG (edit to match your wiring) =================
const uint8_t PIN_ML_A = 5;   // left motor  pin A (PWM)
const uint8_t PIN_ML_B = 6;   // left motor  pin B (PWM)
const uint8_t PIN_MR_A = 9;   // right motor pin A (PWM)
const uint8_t PIN_MR_B = 10;  // right motor pin B (PWM)

const uint8_t PIN_SERVO_L = 2;  // left gripper servo signal  (yellow wire on D2)
const uint8_t PIN_SERVO_R = 3;  // right gripper servo signal (yellow wire on D3)

// Sensor OUT1..OUT5, left -> right
const uint8_t SENSOR_PINS[5] = {A0, A1, A2, A3, A4};
// ===========================================================================

// ---- Tuning ----
const bool    TEST_MODE        = false;  // true = bench test, robot does not drive on the line
const bool    LINE_IS_BLACK    = true;   // sensor reads HIGH on black? flip if reversed
const int     BASE_SPEED       = 140;    // 0..255
const int     TURN_SPEED       = 150;
const float   KP               = 28.0;   // steering gain
const float   KD               = 18.0;
const int     SPIN_180_MS      = 650;    // time to spin ~180 deg  (tune!)
const int     FORWARD_NUDGE_MS = 250;    // creep forward before gripping
const int     BACKUP_MS        = 500;
const int     JUNCTION_NUDGE_MS= 220;    // drive forward so the wheels sit on the junction
const int     TURN_MIN_MS      = 250;    // spin at least this long for 90 deg turns
const int     DEADEND_MS       = 350;    // line missing this long = dead end / cube

// One entry per cube: turns taken at each junction on the way OUT ('L','R','S'=straight)
const char* TRIPS[] = { "S", "L", "R" };
const int   NUM_TRIPS = sizeof(TRIPS) / sizeof(TRIPS[0]);

// Servo angles (tune so the arms don't crush the cube)
const int L_OPEN = 20,  L_CLOSED = 80;
const int R_OPEN = 160, R_CLOSED = 100;   // mirrored

Servo servoL, servoR;

bool carrying = false;
int  tripIdx = 0, tripPos = 0;
char history[16];          // turns taken on the way out
int  histLen = 0;
unsigned long lostSince = 0;

float lastError = 0;

// ---------------- Motors ----------------
void setMotor(uint8_t a, uint8_t b, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(a, speed); analogWrite(b, 0); }
  else            { analogWrite(a, 0);     analogWrite(b, -speed); }
}
void drive(int left, int right) {
  setMotor(PIN_ML_A, PIN_ML_B, left);
  setMotor(PIN_MR_A, PIN_MR_B, right);
}
void stopMotors() { drive(0, 0); }

// ---------------- Grippers ----------------
void openGrip()  { servoL.write(L_OPEN);   servoR.write(R_OPEN);   delay(500); }
void closeGrip() { servoL.write(L_CLOSED); servoR.write(R_CLOSED); delay(600); }

// ---------------- Sensors ----------------
bool readSensor(int i) {
  bool v = digitalRead(SENSOR_PINS[i]);
  return LINE_IS_BLACK ? v : !v;   // true = line seen
}
uint8_t readMask() {
  uint8_t m = 0;
  for (int i = 0; i < 5; i++) if (readSensor(i)) m |= (1 << i);
  return m;
}
// weighted position: -4 (far left) .. +4 (far right); returns false if no line
bool linePosition(float &pos) {
  static const int w[5] = {-4, -2, 0, 2, 4};
  int sum = 0, n = 0;
  for (int i = 0; i < 5; i++) if (readSensor(i)) { sum += w[i]; n++; }
  if (n == 0) return false;
  pos = (float)sum / n;
  return true;
}
bool allBlack() { return readMask() == 0b11111; }
int  countOn() { int n = 0; for (int i = 0; i < 5; i++) if (readSensor(i)) n++; return n; }

// One PD line-following step
void followStep() {
  float pos;
  if (!linePosition(pos)) {
    // lost line: keep turning toward the side we last saw it
    if (lastError < 0) drive(-TURN_SPEED, TURN_SPEED);
    else               drive(TURN_SPEED, -TURN_SPEED);
    return;
  }
  float correction = KP * pos + KD * (pos - lastError);
  lastError = pos;
  drive(BASE_SPEED + correction, BASE_SPEED - correction);
}

// ---------------- Turning ----------------
void spin(int dir, int minMs) {          // dir: -1 = left, +1 = right
  drive(dir * TURN_SPEED, -dir * TURN_SPEED);
  delay(minMs);
  unsigned long t = millis();
  while (!readSensor(2) && millis() - t < 2500) {}   // until centre sensor finds line
  stopMotors(); lastError = 0; delay(100);
}
void doTurn(char t) {
  if (t == 'L') spin(-1, TURN_MIN_MS);
  else if (t == 'R') spin(+1, TURN_MIN_MS);
  // 'S' = straight: just keep going
}
void uTurn() { spin(+1, SPIN_180_MS); }

// ---------------- Test mode ----------------
void testLoop() {
  Serial.print("sensors L->R: ");
  for (int i = 0; i < 5; i++) Serial.print(readSensor(i) ? '#' : '.');
  Serial.println();

  static unsigned long t = 0;
  if (millis() - t > 3000) {
    t = millis();
    Serial.println("test: left fwd");  drive(150, 0);  delay(500); stopMotors();
    Serial.println("test: right fwd"); drive(0, 150);  delay(500); stopMotors();
    Serial.println("test: grippers");  openGrip(); closeGrip(); openGrip();
  }
  delay(100);
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);
  pinMode(PIN_ML_A, OUTPUT); pinMode(PIN_ML_B, OUTPUT);
  pinMode(PIN_MR_A, OUTPUT); pinMode(PIN_MR_B, OUTPUT);
  servoL.attach(PIN_SERVO_L);
  servoR.attach(PIN_SERVO_R);
  openGrip();
  stopMotors();
  delay(1500);   // time to place the robot on the line
}

void handleJunction() {
  stopMotors();
  if (carrying) {
    if (histLen == 0) {                         // back at the start: drop zone
      stopMotors();
      openGrip();
      drive(-BASE_SPEED, -BASE_SPEED); delay(BACKUP_MS); stopMotors();
      carrying = false; tripIdx++; tripPos = 0;
      if (tripIdx >= NUM_TRIPS) { while (true) stopMotors(); }   // all cubes collected
      uTurn();
      return;
    }
    // replay history backwards, mirrored (L<->R)
    char t = history[--histLen];
    t = (t == 'L') ? 'R' : (t == 'R') ? 'L' : 'S';
    drive(BASE_SPEED, BASE_SPEED); delay(JUNCTION_NUDGE_MS); stopMotors();
    doTurn(t);
  } else {
    char t = TRIPS[tripIdx][tripPos] ? TRIPS[tripIdx][tripPos++] : 'S';
    if (histLen < (int)sizeof(history)) history[histLen++] = t;
    drive(BASE_SPEED, BASE_SPEED); delay(JUNCTION_NUDGE_MS); stopMotors();
    doTurn(t);
  }
}

void loop() {
  if (TEST_MODE) { testLoop(); return; }

  uint8_t mask = readMask();

  if (mask == 0) {                              // no line under the robot
    if (lostSince == 0) lostSince = millis();
    if (!carrying && millis() - lostSince > DEADEND_MS) {   // dead end = cube here
      stopMotors();
      drive(BASE_SPEED, BASE_SPEED); delay(FORWARD_NUDGE_MS); stopMotors();
      closeGrip();
      carrying = true;
      uTurn();
      lostSince = 0;
      return;
    }
    followStep();                               // brief gap: keep searching
    return;
  }
  lostSince = 0;

  if (countOn() >= 4) { handleJunction(); return; }
  followStep();
}
