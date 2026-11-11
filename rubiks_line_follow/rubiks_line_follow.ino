/*
  Rubik's collector - simple LINE FOLLOWING version
  Arduino Nano + MNBOT Motor driver, 2 DC motors, 5-channel IR line sensor.

  The robot follows the black lines. The cubes get caught between the two fixed arms
  (the yellow U) and are carried/pushed along the line.
  At every intersection it takes the next move from PLAN:
      S = straight   L = left   R = right   U = U-turn   X = stop
  Example: "SSLSRX" = straight, straight, left, straight, right, stop.

  Check the pins before uploading!
*/

// ===== PINS (edit to match your wiring) =====
const uint8_t ML_A = 5, ML_B = 6;      // left motor
const uint8_t MR_A = 9, MR_B = 10;     // right motor
const uint8_t SENSOR[5] = {A0, A1, A2, A3, A4};   // OUT1..OUT5, left -> right

// ===== YOUR ROUTE =====
const char PLAN[] = "SSLSRSX";

// ===== TUNING =====
const bool  LINE_IS_BLACK = true;   // flip if robot runs away from the line
const int   SPEED       = 130;      // 0..255 (go slower so cubes don't get knocked out)
const int   TURN_FAST   = 130;      // turns: outer motor speed
const int   TURN_SLOW   = 20;       // turns: inner motor speed (0 = pivot on one wheel, no reverse)
const float KP = 28.0, KD = 18.0;
const int   NUDGE_MS    = 220;      // roll onto the intersection before turning
const int   TURN_MS     = 450;      // minimum arc time for 90 deg (tune!)
const int   UTURN_MS    = 600;      // in-place spin time for 180 deg (tune!)
const int   UTURN_SPEED = 130;      // in-place spin speed
const int   DEBOUNCE_MS = 300;      // ignore intersections right after one
const int   DEADEND_MS  = 400;      // no line this long = dead end -> U-turn

int   planPos = 0;
float lastPos = 0;
unsigned long lastNode = 0, lostSince = 0;

void setMotor(uint8_t a, uint8_t b, int s) {
  s = constrain(s, -255, 255);
  if (s >= 0) { analogWrite(a, s); analogWrite(b, 0); }
  else        { analogWrite(a, 0); analogWrite(b, -s); }
}
void drive(int l, int r) { setMotor(ML_A, ML_B, l); setMotor(MR_A, MR_B, r); }

bool on(int i) { bool v = digitalRead(SENSOR[i]); return LINE_IS_BLACK ? v : !v; }
int  countOn() { int n = 0; for (int i = 0; i < 5; i++) n += on(i); return n; }
bool intersection() {
  return countOn() >= 4 || (on(0) && on(1) && on(2)) || (on(2) && on(3) && on(4));
}

// Turn WITHOUT rotating in place: one motor slower, the other faster (both go forward).
void spin(int dir, int minMs) {              // -1 left, +1 right
  if (dir < 0) drive(TURN_SLOW, TURN_FAST);  // left turn: left motor slower
  else         drive(TURN_FAST, TURN_SLOW);  // right turn: right motor slower
  delay(minMs);
  unsigned long t = millis();
  while (!on(2) && millis() - t < 2500) {}   // until centre sensor finds the new line
  drive(0, 0); lastPos = 0; delay(100);
}

// U-turn: rotate IN PLACE (one wheel forward, the other backward).
void uTurn(int minMs) {
  drive(UTURN_SPEED, -UTURN_SPEED);
  delay(minMs);
  unsigned long t = millis();
  while (!on(2) && millis() - t < 2500) {}   // until the centre sensor finds the line
  drive(0, 0); lastPos = 0; delay(100);
}

void followLine() {
  static const int w[5] = {-4, -2, 0, 2, 4};
  int sum = 0, n = 0;
  for (int i = 0; i < 5; i++) if (on(i)) { sum += w[i]; n++; }
  if (n == 0) {                              // lost the line: search toward last side
    if (lastPos < 0) drive(TURN_SLOW, TURN_FAST); else drive(TURN_FAST, TURN_SLOW);
    return;
  }
  float pos = (float)sum / n;
  float c = KP * pos + KD * (pos - lastPos);
  lastPos = pos;
  drive(SPEED + c, SPEED - c);
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 5; i++) pinMode(SENSOR[i], INPUT);
  pinMode(ML_A, OUTPUT); pinMode(ML_B, OUTPUT);
  pinMode(MR_A, OUTPUT); pinMode(MR_B, OUTPUT);
  delay(2000);                               // put the robot on the line
}

void loop() {
  // dead end: line gone for a while -> turn around
  if (countOn() == 0) {
    if (!lostSince) lostSince = millis();
    if (millis() - lostSince > DEADEND_MS) { uTurn(UTURN_MS); lostSince = 0; return; }
  } else lostSince = 0;

  if (millis() - lastNode > DEBOUNCE_MS && intersection()) {
    drive(SPEED, SPEED); delay(NUDGE_MS); drive(0, 0);
    char m = PLAN[planPos];
    if (m == '\0' || m == 'X') { while (true) drive(0, 0); }   // finished
    planPos++;
    if (m == 'L') spin(-1, TURN_MS);
    else if (m == 'R') spin(+1, TURN_MS);
    else if (m == 'U') uTurn(UTURN_MS);
    lastNode = millis();
    return;
  }
  followLine();
}
