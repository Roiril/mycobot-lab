// Hiwonder 5-finger robot hand (LFD-01 x5) — independent finger control
// Board: Arduino Uno
// Power: servos from EXTERNAL 6V supply. Arduino GND <-> 6V GND (common ground).
//        Do NOT power servos from the Arduino 5V pin.
//
// Serial usage (9600 baud, Newline):
//   "0 2000"            -> set finger 0 to 2000us (blocking gentle ramp)
//   "open"              -> all fingers to OPEN preset (blocking)
//   "close"             -> all fingers to CLOSE preset (blocking)
//   "n"                 -> all fingers to NEUTRAL (1500us, blocking)
//   "spd <step>"        -> blocking-ramp step size (gentler current)
//   "t <u0> <u1> <u2> <u3> <u4>"  -> TELEOP: set all 5 targets, NON-BLOCKING.
//                          loop() ramps toward targets without blocking on
//                          serial, so a ~25Hz stream of 't' lines never piles
//                          up. Silent (no ack) to keep the bus clean.
//   "tspd <step> <ms>"  -> teleop ramp tuning (us/step, ms/step). Smaller =
//                          gentler current but slower follow.

#include <Servo.h>

const int NUM_FINGERS = 5;

// finger index -> Arduino signal pin
const int PINS[NUM_FINGERS] = {3, 5, 6, 9, 10};

// Per-finger safe range in microseconds.
// Validated at 6V (4xAA): thumb sweeps 1000-2000 with no strain; fingers 1-4
// open/close at 600/2400 ran clean. Tighten any finger here if it ever stalls.
//                          finger:   0(thumb) 1     2     3     4
const int MIN_US[NUM_FINGERS] = {     1000,   600,  600,  600,  600 };
const int MAX_US[NUM_FINGERS] = {     2000,  2400, 2400, 2400, 2400 };

// "open" / "close" presets per finger (must stay within MIN/MAX above)
const int OPEN_US[NUM_FINGERS]  = {   2000,  2400, 2400, 2400, 2400 };
const int CLOSE_US[NUM_FINGERS] = {   1000,   600,  600,  600,  600 };

Servo servos[NUM_FINGERS];
int curUs[NUM_FINGERS];   // last commanded position per finger
int tgtUs[NUM_FINGERS];   // teleop target per finger (loop ramps toward this)

// Blocking smooth-move tuning (used by open/close/n/<f> <us>):
// smaller STEP / larger DELAY = gentler current draw.
int STEP_US = 8;          // microseconds per step
int STEP_MS = 12;         // delay between steps

// Teleop (non-blocking) ramp tuning. Faster than the blocking ramp so live
// hand-tracking follows responsively, but still stepped to bound di/dt.
int TELE_STEP_US = 25;            // us per ramp step
unsigned long TELE_STEP_MS = 8;   // ms between ramp steps
unsigned long lastTeleStepMs = 0;

int clampUs(int finger, int us) {
  if (us < MIN_US[finger]) return MIN_US[finger];
  if (us > MAX_US[finger]) return MAX_US[finger];
  return us;
}

// Immediate (no ramp). Used at boot only.
void setFingerNow(int finger, int us) {
  if (finger < 0 || finger >= NUM_FINGERS) return;
  us = clampUs(finger, us);
  servos[finger].writeMicroseconds(us);
  curUs[finger] = us;
  tgtUs[finger] = us;
}

// Ramp one finger from its current position to target in small steps (blocking).
void smoothFinger(int finger, int target) {
  if (finger < 0 || finger >= NUM_FINGERS) return;
  target = clampUs(finger, target);
  int cur = curUs[finger];
  while (abs(target - cur) > STEP_US) {
    cur += (target > cur) ? STEP_US : -STEP_US;
    servos[finger].writeMicroseconds(cur);
    delay(STEP_MS);
  }
  servos[finger].writeMicroseconds(target);
  curUs[finger] = target;
  tgtUs[finger] = target;   // keep teleop target in sync so loop() won't fight
}

// Ramp all fingers toward their targets together, interleaved one small
// step at a time (blocking). Spreads current over time.
void smoothAll(const int target[NUM_FINGERS]) {
  int tgt[NUM_FINGERS];
  for (int i = 0; i < NUM_FINGERS; i++) tgt[i] = clampUs(i, target[i]);
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < NUM_FINGERS; i++) {
      int cur = curUs[i];
      if (abs(tgt[i] - cur) > STEP_US) {
        cur += (tgt[i] > cur) ? STEP_US : -STEP_US;
        servos[i].writeMicroseconds(cur);
        curUs[i] = cur;
        moving = true;
      } else if (cur != tgt[i]) {
        servos[i].writeMicroseconds(tgt[i]);
        curUs[i] = tgt[i];
      }
    }
    delay(STEP_MS);
  }
  for (int i = 0; i < NUM_FINGERS; i++) tgtUs[i] = tgt[i];  // sync teleop target
}

// One non-blocking ramp tick toward tgtUs[]. Called every loop iteration; only
// actually steps when TELE_STEP_MS has elapsed. Never blocks.
void teleopTick() {
  unsigned long now = millis();
  if (now - lastTeleStepMs < TELE_STEP_MS) return;
  lastTeleStepMs = now;
  for (int i = 0; i < NUM_FINGERS; i++) {
    int cur = curUs[i];
    int t = tgtUs[i];
    if (cur == t) continue;
    int d = t - cur;
    if (abs(d) <= TELE_STEP_US) cur = t;
    else cur += (d > 0) ? TELE_STEP_US : -TELE_STEP_US;
    servos[i].writeMicroseconds(cur);
    curUs[i] = cur;
  }
}

// Parse "t u0 u1 u2 u3 u4" into tgtUs[] (clamped). Returns true on success.
bool parseTeleop(const String &line) {
  int idx = 1;  // start after 't'
  for (int f = 0; f < NUM_FINGERS; f++) {
    while (idx < (int)line.length() && line[idx] == ' ') idx++;
    if (idx >= (int)line.length()) return false;
    int start = idx;
    while (idx < (int)line.length() && line[idx] != ' ') idx++;
    int v = line.substring(start, idx).toInt();
    tgtUs[f] = clampUs(f, v);
  }
  return true;
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_FINGERS; i++) {
    servos[i].attach(PINS[i]);
    setFingerNow(i, 1500); // neutral on boot
  }
  Serial.println(F("ready. '<f> <us>', 'open'/'close'/'n', 'spd <step>', 't u0..u4', 'tspd <us> <ms>'"));
}

void loop() {
  // Always service the non-blocking teleop ramp first so follow stays smooth
  // even while waiting for the next serial line.
  teleopTick();

  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // --- TELEOP (non-blocking, silent): "t u0 u1 u2 u3 u4" ---
  if (line == "t" || line.startsWith("t ")) {
    parseTeleop(line);   // updates tgtUs; loop ramps toward it. No ack.
    return;
  }
  if (line.startsWith("tspd ")) {
    int sp = line.indexOf(' ', 5);
    if (sp > 0) {
      int us = line.substring(5, sp).toInt();
      int ms = line.substring(sp + 1).toInt();
      if (us >= 1) TELE_STEP_US = us;
      if (ms >= 1) TELE_STEP_MS = (unsigned long)ms;
    }
    Serial.print(F("tele step=")); Serial.print(TELE_STEP_US);
    Serial.print(F("us / ")); Serial.print(TELE_STEP_MS); Serial.println(F("ms"));
    return;
  }

  if (line == "open")  { smoothAll(OPEN_US);  Serial.println(F("open"));  return; }
  if (line == "close") { smoothAll(CLOSE_US); Serial.println(F("close")); return; }
  if (line == "n") {
    int mid[NUM_FINGERS];
    for (int i = 0; i < NUM_FINGERS; i++) mid[i] = 1500;
    smoothAll(mid);
    Serial.println(F("neutral"));
    return;
  }

  int sp = line.indexOf(' ');
  if (sp <= 0) { Serial.println(F("format: <finger> <us>")); return; }

  // "spd <step>" : set blocking ramp step size (smaller = gentler)
  if (line.startsWith("spd ")) {
    STEP_US = line.substring(4).toInt();
    if (STEP_US < 1) STEP_US = 1;
    Serial.print(F("step=")); Serial.print(STEP_US);
    Serial.print(F("us / ")); Serial.print(STEP_MS); Serial.println(F("ms"));
    return;
  }

  int finger = line.substring(0, sp).toInt();
  int us = line.substring(sp + 1).toInt();
  smoothFinger(finger, us);
  Serial.print(F("finger ")); Serial.print(finger);
  Serial.print(F(" -> ")); Serial.print(clampUs(finger, us));
  Serial.println(F("us"));
}
