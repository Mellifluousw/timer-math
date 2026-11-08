/*
  Rubik's Cube Collector Robot  (Arduino Nano + MNBOT Motor driver v1.1)

  Hardware (from photos):
    - 2 x DC gear motors (left/right wheels) on the MNBOT driver (ZM / BM terminals)
    - 2 x MG90S servos = left & right gripper arms
    - 5-channel IR line sensor array (GND, 5V, OUT1..OUT5)

  Behaviour:
    1. Follow the line.
    2. When ALL sensors see black (cube/stop marker) -> stop, close the grippers on the cube.
    3. Turn around 180 deg, follow the line back.
    4. At the next all-black marker (drop zone) -> open grippers, back up, stop.

  !!! CHECK THE PIN BLOCK BELOW AGAINST YOUR WIRING BEFORE UPLOADING !!!
  Use TEST_MODE = true first: it prints sensors to Serial and wiggles motors/servos.
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

// Servo angles (tune so the arms don't crush the cube)
const int L_OPEN = 20,  L_CLOSED = 80;
const int R_OPEN = 160, R_CLOSED = 100;   // mirrored

Servo servoL, servoR;

enum State { FOLLOW_TO_CUBE, GRAB, TURN_AROUND, FOLLOW_TO_DROP, RELEASE, DONE };
State state = FOLLOW_TO_CUBE;

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

void loop() {
  if (TEST_MODE) { testLoop(); return; }

  switch (state) {
    case FOLLOW_TO_CUBE:
      if (allBlack()) { stopMotors(); state = GRAB; }
      else followStep();
      break;

    case GRAB:
      drive(BASE_SPEED, BASE_SPEED); delay(FORWARD_NUDGE_MS);  // center on the cube
      stopMotors();
      closeGrip();
      state = TURN_AROUND;
      break;

    case TURN_AROUND:
      drive(-TURN_SPEED, TURN_SPEED); delay(SPIN_180_MS);
      stopMotors();
      // keep spinning until the middle sensor re-finds the line
      while (!readSensor(2)) drive(-TURN_SPEED, TURN_SPEED);
      stopMotors();
      lastError = 0;
      delay(150);
      state = FOLLOW_TO_DROP;
      break;

    case FOLLOW_TO_DROP:
      if (allBlack()) { stopMotors(); state = RELEASE; }
      else followStep();
      break;

    case RELEASE:
      openGrip();
      drive(-BASE_SPEED, -BASE_SPEED); delay(BACKUP_MS);
      stopMotors();
      state = DONE;
      break;

    case DONE:
      stopMotors();
      break;
  }
}
