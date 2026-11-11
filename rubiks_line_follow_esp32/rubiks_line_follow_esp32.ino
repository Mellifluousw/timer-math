/*
  Rubik's collector - simple LINE FOLLOWING version  (ESP32)
  ESP32 DevKit + motor driver (L298N / TB6612 / DRV8833 style, 2 inputs per motor),
  2 DC motors, 5-channel IR line sensor.

  NOTE: ESP32 GPIOs are 3.3 V. Power the sensor board from 3.3 V (or make sure its OUT pins
  never exceed 3.3 V). The Nano-based MNBOT board is not used with the ESP32 - wire your
  motor driver's input pins to the GPIOs below.

  The robot follows the black lines. The cubes get caught between the two fixed arms
  (the yellow U) and are carried/pushed along the line.
  At every intersection it takes the next move from PLAN:
      S = straight   L = left   R = right   U = U-turn   X = stop
  Example: "SSLSRX" = straight, straight, left, straight, right, stop.

  Check the pins before uploading!
*/

// ===== PINS (ESP32 GPIO numbers - edit to match your wiring) =====
const uint8_t ML_A = 25, ML_B = 26;    // left motor
const uint8_t MR_A = 27, MR_B = 14;    // right motor
// OUT1..OUT5, left -> right. 36/39/34/35 are input-only pins: perfect for sensors.
const uint8_t SENSOR[5] = {36, 39, 34, 35, 32};

// ===== YOUR ROUTE =====
const char PLAN[] = "SSLSRSX";

// ===== TUNING =====
const bool  LINE_IS_BLACK = true;   // flip if robot runs away from the line
const int   SPEED       = 130;      // 0..255 (go slower so cubes don't get knocked out)
const int   TURN_SPEED  = 120;
const float KP = 28.0, KD = 18.0;
const int   NUDGE_MS    = 220;      // roll onto the intersection before turning
const int   TURN_MS     = 250;      // minimum spin time for 90 deg
const int   UTURN_MS    = 600;      // minimum spin time for 180 deg
const int   DEBOUNCE_MS = 300;      // ignore intersections right after one
const int   DEADEND_MS  = 400;      // no line this long = dead end -> U-turn

int   planPos = 0;
float lastPos = 0;
unsigned long lastNode = 0, lostSince = 0;

// PWM: ESP32 Arduino core 3.x has analogWrite(); core 2.x needs LEDC channels.
const int PWM_FREQ = 5000, PWM_BITS = 8;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  void pwmBegin(uint8_t pin, int ch) { ledcAttach(pin, PWM_FREQ, PWM_BITS); }
  void pwmWrite(uint8_t pin, int ch, int duty) { ledcWrite(pin, duty); }
#else
  void pwmBegin(uint8_t pin, int ch) { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); }
  void pwmWrite(uint8_t pin, int ch, int duty) { ledcWrite(ch, duty); }
#endif

void setMotor(uint8_t a, uint8_t b, int chA, int s) {
  s = constrain(s, -255, 255);
  if (s >= 0) { pwmWrite(a, chA, s); pwmWrite(b, chA + 1, 0); }
  else        { pwmWrite(a, chA, 0); pwmWrite(b, chA + 1, -s); }
}
void drive(int l, int r) { setMotor(ML_A, ML_B, 0, l); setMotor(MR_A, MR_B, 2, r); }

bool on(int i) { bool v = digitalRead(SENSOR[i]); return LINE_IS_BLACK ? v : !v; }
int  countOn() { int n = 0; for (int i = 0; i < 5; i++) n += on(i); return n; }
bool intersection() {
  return countOn() >= 4 || (on(0) && on(1) && on(2)) || (on(2) && on(3) && on(4));
}

void spin(int dir, int minMs) {              // -1 left, +1 right
  drive(dir * TURN_SPEED, -dir * TURN_SPEED);
  delay(minMs);
  unsigned long t = millis();
  while (!on(2) && millis() - t < 2500) {}   // until centre sensor finds the new line
  drive(0, 0); lastPos = 0; delay(100);
}

void followLine() {
  static const int w[5] = {-4, -2, 0, 2, 4};
  int sum = 0, n = 0;
  for (int i = 0; i < 5; i++) if (on(i)) { sum += w[i]; n++; }
  if (n == 0) {                              // lost the line: search toward last side
    if (lastPos < 0) drive(-TURN_SPEED, TURN_SPEED); else drive(TURN_SPEED, -TURN_SPEED);
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
  pwmBegin(ML_A, 0); pwmBegin(ML_B, 1);
  pwmBegin(MR_A, 2); pwmBegin(MR_B, 3);
  drive(0, 0);
  delay(2000);                               // put the robot on the line
}

void loop() {
  // dead end: line gone for a while -> turn around
  if (countOn() == 0) {
    if (!lostSince) lostSince = millis();
    if (millis() - lostSince > DEADEND_MS) { spin(+1, UTURN_MS); lostSince = 0; return; }
  } else lostSince = 0;

  if (millis() - lastNode > DEBOUNCE_MS && intersection()) {
    drive(SPEED, SPEED); delay(NUDGE_MS); drive(0, 0);
    char m = PLAN[planPos];
    if (m == '\0' || m == 'X') { while (true) drive(0, 0); }   // finished
    planPos++;
    if (m == 'L') spin(-1, TURN_MS);
    else if (m == 'R') spin(+1, TURN_MS);
    else if (m == 'U') spin(+1, UTURN_MS);
    lastNode = millis();
    return;
  }
  followLine();
}
