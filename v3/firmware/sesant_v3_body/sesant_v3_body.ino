// SesAnt v3: RealAnt-protocol bridge + home puck + StackChan neck + ant mandibles
// ---------------------------------------------------------------------------
// Builds on v2 (firmware v2/firmware/sesant_v2_puck): same 8 leg servos, battery monitor, dock / undock on the home puck.
// New in v3:
//   * neck : 2x Feetech SCS0009 serial servos (StackChan's own neck servos), IDs 1 = pan, 2 = tilt, on a half-duplex
//            bus on Serial1 (TX GPIO 17 -> 1 kohm -> bus, RX GPIO 18 straight to the bus), 1 Mbit/s, SCS protocol
//            (big-endian registers, 0.3125 deg/step as in StackChan/firmware/main/hal/hal_servo.cpp).
//   * jaw  : 1x MG90S on GPIO 7 drives the left mandible directly; a printed gear pair mirrors it onto the right one.
//   * link : the CoreS3 in the head can send the same command lines over UART (Serial0, RX GPIO 37 / TX GPIO 39).
// Commands (in addition to v1/v2: attach_servos, detach_servos, reset, standup, sN <pos>, dock, undock, battery):
//   neck <pan_deg> <tilt_deg>   pan -120..120 (+ = left), tilt -20..60 (+ = look down); measured angles go into meas
//   neck_zero                   store the current neck positions as 0/0 (after mounting the head straight and level)
//   neck_torque 0|1
//   jaw <deg>                   mandible opening per side, 0 = closed .. 40 = open
//   grab | release              jaw 0 | jaw 40
//   crouch                      grab pose from cad/scripts/v3_neck_jaw.py: knees out, belly cone 1.5 mm above the floor,
//                               open mandibles 2 mm above the floor, head looking down at them
//   paw                         shift onto FR/BR/BL, lift FL forward and tap twice, then stand up again
// meas line: the 29 RealAnt fields + free heap, then  vbat_mV charge_mV dock_state  (v2)  then  pan_deg tilt_deg jaw_deg
//            (neck angles measured by the SCS0009, -999 if a servo does not answer). ant_server.py reads the first 29.
// Joint convention as v1: s1..s8 = FL hip, FL knee, BL hip, BL knee, BR hip, BR knee, FR hip, FR knee (AX units);
// knee 224 = foot vertical, + = foot outward.

#include <ESP32Servo.h>
#include <Preferences.h>

const int servoPins[8] = {1, 2, 4, 6, 8, 10, 13, 14};
enum SesameCh : uint8_t { R1 = 0, R2 = 1, L1 = 2, L2 = 3, R4 = 4, R3 = 5, L3 = 6, L4 = 7 };
#define VBAT_PIN 3
#define CHG_PIN 5
#define JAW_PIN 7
#define BUS_TX 17
#define BUS_RX 18
#define LINK_RX 37
#define LINK_TX 39
const float DIVIDER = (100.0f + 27.0f) / 27.0f;

const uint8_t REALANT_TO_CH[8] = {L1, L3, L2, L4, R2, R4, R1, R3};
int8_t DIR[8] = {+1, -1, +1, -1, +1, +1, +1, +1};
int8_t TRIM[8] = {0, 0, 0, 0, 0, 0, 0, 0};

const float AX_DEG_PER_UNIT = 300.0f / 1023.0f;
const int   AX_CENTER = 512;
const int   KNEE_STAND_UNITS = 224;
const int   KNEE_DOCK_UNITS = 396 + 7;                 // v2 seat pose + 2 deg
const int   KNEE_CROUCH_FRONT = 446, KNEE_CROUCH_BACK = 447;   // v3 grab pose (65 deg outward)
const float SERVO_DEG_PER_MS = 0.6f;
const int   HIP_LIMIT_DEG = 88;
const int   KNEE_MIN_DEG = -48, KNEE_MAX_DEG = 118;
const unsigned long DOCK_RAMP_MS = 2000, SETTLE_MS = 400;
const int   CHG_PRESENT_MV = 6000, VBAT_LOW_MV = 6900;

// paw keyframes (CAD hip angles from v3_neck_jaw.py: FR +40, BL -45, FL -20; FL knee 95 deg out).
// HIP_SIGN maps CAD hip degrees (+ = counter-clockwise from above) to RealAnt hip units: flip it if `paw` swings the
// support legs the wrong way during calibration.
const int   HIP_SIGN = +1;
const int   PAW_HIP_FR = 40, PAW_HIP_BL = -45, PAW_HIP_FL = -20;
const int   PAW_KNEE_UP = 224 + 324, PAW_KNEE_TAP = 224 + 205;   // 95 / 60 deg outward

// neck
const int   PAN_MIN = -120, PAN_MAX = 120, TILT_MIN = -20, TILT_MAX = 60, CROUCH_TILT = 60;
const float SCS_STEP_PER_DEG = 3.2f;                   // StackChan hal_servo.cpp: 0.3125 deg per step
int   neckZero[2] = {512, 512};
int   neckSign[2] = {+1, -1};                          // tilt: + = look down (check once, flip in neck_zero if needed)
float neckMeas[2] = {-999, -999};
// jaw
const int   JAW_OPEN_DEG = 40, JAW_DIR = +1, JAW_TRIM = 0;

enum DockState : uint8_t { ST_FREE = 0, ST_DOCKING = 1, ST_DOCKED = 2, ST_CHARGING = 3, ST_UNDOCKING = 4, ST_DOCK_FAIL = 5 };

Servo servos[8], jawServo;
Preferences prefs;
HardwareSerial &BUS = Serial1;
bool attached = false;
int setpoint[8] = {512, 224, 512, 224, 512, 224, 512, 224};
float estimate[8] = {512, 224, 512, 224, 512, 224, 512, 224};
String usbLine, linkLine;
unsigned long lastMeas = 0, lastEst = 0, lastLowWarn = 0, lastNeckRead = 0, settleT0 = 0;
uint8_t state = ST_FREE;
int vbat_mV = 0, chg_mV = 0, jawDeg = 0;

// --- keyframe player: queue of joint targets, each reached with a cosine ramp
struct Key { int16_t q[8]; uint16_t ms; };
Key keys[10]; uint8_t nKeys = 0, curKey = 0;
struct Ramp { bool active; int from[8], to[8]; unsigned long t0, dur; uint8_t next; } ramp = {false};

static inline bool isKnee(int i) { return (i % 2) == 1; }
int hipUnits(int cadDeg) { return AX_CENTER + (int)lroundf(HIP_SIGN * cadDeg / AX_DEG_PER_UNIT); }

int jointDeg(int i, int ax) {
  float a = isKnee(i) ? (ax - KNEE_STAND_UNITS) * AX_DEG_PER_UNIT : (ax - AX_CENTER) * AX_DEG_PER_UNIT;
  if (isKnee(i)) a = constrain(a, (float)KNEE_MIN_DEG, (float)KNEE_MAX_DEG);
  else a = constrain(a, (float)-HIP_LIMIT_DEG, (float)HIP_LIMIT_DEG);
  return (int)lroundf(a);
}
void writeJoint(int i) {
  if (!attached) return;
  servos[REALANT_TO_CH[i]].write(constrain(90 + DIR[i] * jointDeg(i, setpoint[i]) + TRIM[i], 0, 180));
}
void attachAll() {
  for (int i = 0; i < 8; i++) { servos[i].setPeriodHertz(50); servos[i].attach(servoPins[i], 732, 2929); }
  attached = true;
  for (int i = 0; i < 8; i++) writeJoint(i);
}
void detachAll() { for (int i = 0; i < 8; i++) servos[i].detach(); attached = false; }

void startRamp(const int target[8], unsigned long dur, uint8_t next) {
  for (int i = 0; i < 8; i++) { ramp.from[i] = setpoint[i]; ramp.to[i] = target[i]; }
  ramp.t0 = millis(); ramp.dur = dur; ramp.next = next; ramp.active = true;
}
void playKeys() { curKey = 0; if (nKeys) { int t[8]; for (int i = 0; i < 8; i++) t[i] = keys[0].q[i]; startRamp(t, keys[0].ms, 255); } }
void addKey(const int q[8], uint16_t ms) { if (nKeys < 10) { for (int i = 0; i < 8; i++) keys[nKeys].q[i] = q[i]; keys[nKeys].ms = ms; nKeys++; } }
void poseTargets(int kneeUnits, int out[8]) { for (int i = 0; i < 8; i++) out[i] = isKnee(i) ? kneeUnits : AX_CENTER; }

void readPower() {
  vbat_mV = (int)(analogReadMilliVolts(VBAT_PIN) * DIVIDER);
  chg_mV = (int)(analogReadMilliVolts(CHG_PIN) * DIVIDER);
}

// --- SCS bus (half duplex: our own bytes come back as an echo and are skipped)
uint8_t scsChk(const uint8_t *p, int n) { uint16_t s = 0; for (int i = 2; i < n; i++) s += p[i]; return ~(uint8_t)s; }
void busSend(const uint8_t *p, int n) {
  while (BUS.available()) BUS.read();
  BUS.write(p, n); BUS.flush();
  unsigned long t0 = micros(); int got = 0;
  while (got < n && micros() - t0 < 2000) if (BUS.available()) { BUS.read(); got++; }
}
int busRecv(uint8_t *buf, int n, unsigned long to_us = 3000) {
  unsigned long t0 = micros(); int got = 0;
  while (got < n && micros() - t0 < to_us) if (BUS.available()) buf[got++] = BUS.read();
  return got;
}
void scsWrite(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len) {
  uint8_t p[16] = {0xFF, 0xFF, id, (uint8_t)(len + 3), 0x03, addr};
  for (int i = 0; i < len; i++) p[6 + i] = data[i];
  p[6 + len] = scsChk(p, 6 + len);
  busSend(p, 7 + len);
  uint8_t ack[6]; busRecv(ack, 6);                     // status packet (ignored)
}
int scsReadPos(uint8_t id) {
  uint8_t p[8] = {0xFF, 0xFF, id, 4, 0x02, 56, 2, 0};
  p[7] = scsChk(p, 7); busSend(p, 8);
  uint8_t r[8];
  if (busRecv(r, 8) != 8 || r[0] != 0xFF || r[2] != id || r[7] != scsChk(r, 7)) return -1;
  return (r[5] << 8) | r[6];                           // SCSCL is big-endian
}
void scsGoal(uint8_t id, int pos, int ms) {
  pos = constrain(pos, 0, 1023);
  uint8_t d[6] = {(uint8_t)(pos >> 8), (uint8_t)pos, (uint8_t)(ms >> 8), (uint8_t)ms, 0, 0};
  scsWrite(id, 42, d, 6);
}
void neckTorque(bool on) { uint8_t v = on; scsWrite(1, 40, &v, 1); scsWrite(2, 40, &v, 1); }
void neckTo(float pan, float tilt, int ms = 300) {
  pan = constrain(pan, (float)PAN_MIN, (float)PAN_MAX); tilt = constrain(tilt, (float)TILT_MIN, (float)TILT_MAX);
  scsGoal(1, neckZero[0] + (int)lroundf(neckSign[0] * pan * SCS_STEP_PER_DEG), ms);
  scsGoal(2, neckZero[1] + (int)lroundf(neckSign[1] * tilt * SCS_STEP_PER_DEG), ms);
}
void readNeck() {
  for (int k = 0; k < 2; k++) {
    int p = scsReadPos(k + 1);
    neckMeas[k] = p < 0 ? -999 : (p - neckZero[k]) / (neckSign[k] * SCS_STEP_PER_DEG);
  }
}

// --- jaw
void jawTo(int deg) {
  jawDeg = constrain(deg, 0, JAW_OPEN_DEG);
  if (!jawServo.attached()) { jawServo.setPeriodHertz(50); jawServo.attach(JAW_PIN, 500, 2400); }
  jawServo.write(constrain(90 + JAW_DIR * jawDeg + JAW_TRIM, 0, 180));
}

// --- macros
void cmdDock() {
  if (!attached) attachAll();
  int t[8]; poseTargets(KNEE_DOCK_UNITS, t);
  neckTo(0, 0); jawTo(0);
  state = ST_DOCKING; startRamp(t, DOCK_RAMP_MS, ST_DOCKED);
  Serial.println("- dock: sitting down on the puck");
}
void cmdUndock() {
  int t[8];
  if (!attached) { poseTargets(KNEE_DOCK_UNITS, setpoint); for (int i = 0; i < 8; i++) estimate[i] = setpoint[i]; attachAll(); }
  poseTargets(KNEE_STAND_UNITS, t);
  state = ST_UNDOCKING; startRamp(t, DOCK_RAMP_MS, ST_FREE);
  Serial.println("- undock: standing up");
}
void cmdCrouch() {
  if (!attached) attachAll();
  int t[8] = {AX_CENTER, KNEE_CROUCH_FRONT, AX_CENTER, KNEE_CROUCH_BACK, AX_CENTER, KNEE_CROUCH_BACK, AX_CENTER, KNEE_CROUCH_FRONT};
  jawTo(JAW_OPEN_DEG); neckTo(0, CROUCH_TILT, 800);
  state = ST_FREE; startRamp(t, 1500, ST_FREE);
  Serial.println("- crouch: mandibles open at the floor, head looking down");
}
void cmdPaw() {
  if (!attached) attachAll();
  nKeys = 0;
  int s[8] = {AX_CENTER, KNEE_STAND_UNITS, AX_CENTER, KNEE_STAND_UNITS, AX_CENTER, KNEE_STAND_UNITS, AX_CENTER, KNEE_STAND_UNITS};
  int a[8]; memcpy(a, s, sizeof a);
  a[6] = hipUnits(PAW_HIP_FR); a[2] = hipUnits(PAW_HIP_BL);                 // support legs: shift the CoM off FL
  addKey(a, 900);
  a[0] = hipUnits(PAW_HIP_FL); a[1] = PAW_KNEE_UP; addKey(a, 600);         // lift FL forward
  for (int r = 0; r < 2; r++) { a[1] = PAW_KNEE_TAP; addKey(a, 250); a[1] = PAW_KNEE_UP; addKey(a, 250); }
  a[0] = AX_CENTER; a[1] = KNEE_STAND_UNITS; addKey(a, 600);               // foot back down
  addKey(s, 900);                                                          // support legs home
  neckTo(0, 20);
  state = ST_FREE; playKeys();
  Serial.println("- paw: on FR/BR/BL, tapping with FL");
}

void handleLine(String line) {
  line.trim();
  if (!line.length()) return;
  Serial.print("- recv: "); Serial.println(line);
  float a = 0, b = 0;
  if (line.startsWith("attach_servos")) { attachAll(); Serial.println("- Succeeded to change torque on"); }
  else if (line.startsWith("detach_servos")) { ramp.active = false; nKeys = 0; detachAll(); Serial.println("- Succeeded to change torque off"); }
  else if (line.startsWith("reset"))   { ramp.active = false; nKeys = 0; for (int i = 0; i < 8; i++) { setpoint[i] = 512; writeJoint(i); } }
  else if (line.startsWith("standup")) { ramp.active = false; nKeys = 0; for (int i = 0; i < 8; i++) { setpoint[i] = isKnee(i) ? KNEE_STAND_UNITS : 512; writeJoint(i); } }
  else if (line.startsWith("undock"))  cmdUndock();
  else if (line.startsWith("dock"))    cmdDock();
  else if (line.startsWith("battery")) { readPower(); Serial.printf("- battery %d %d %d\n", vbat_mV, chg_mV, state); }
  else if (line.startsWith("neck_zero")) {
    for (int k = 0; k < 2; k++) { int p = scsReadPos(k + 1); if (p >= 0) neckZero[k] = p; }
    prefs.putInt("pan0", neckZero[0]); prefs.putInt("tilt0", neckZero[1]);
    Serial.printf("- neck zero %d %d\n", neckZero[0], neckZero[1]);
  }
  else if (line.startsWith("neck_torque")) neckTorque(line.endsWith("1"));
  else if (line.startsWith("neck") && sscanf(line.c_str(), "neck %f %f", &a, &b) == 2) neckTo(a, b);
  else if (line.startsWith("jaw") && sscanf(line.c_str(), "jaw %f", &a) == 1) jawTo((int)a);
  else if (line.startsWith("grab"))    jawTo(0);
  else if (line.startsWith("release")) jawTo(JAW_OPEN_DEG);
  else if (line.startsWith("crouch"))  cmdCrouch();
  else if (line.startsWith("paw"))     cmdPaw();
  else {
    ramp.active = false; nKeys = 0;
    if (state != ST_FREE && attached) state = ST_FREE;
    char buf[260]; line.toCharArray(buf, sizeof(buf));
    char *save = nullptr; char *tok = strtok_r(buf, " \t", &save);
    while (tok) {
      char *val = strtok_r(nullptr, " \t", &save);
      if (!val) break;
      if (tok[0] == 's') { int s = atoi(tok + 1); if (s >= 1 && s <= 8) { setpoint[s - 1] = constrain(atoi(val), 0, 1023); writeJoint(s - 1); } }
      tok = strtok_r(nullptr, " \t", &save);
    }
  }
}

void pump(Stream &in, String &buf) {
  while (in.available()) {
    char c = (char)in.read();
    buf += c;
    if (c == '\n') { handleLine(buf); buf = ""; }
    if (buf.length() > 256) buf = "";
  }
}

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200, SERIAL_8N1, LINK_RX, LINK_TX);
  BUS.begin(1000000, SERIAL_8N1, BUS_RX, BUS_TX);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db); analogSetPinAttenuation(CHG_PIN, ADC_11db);
  prefs.begin("sesant", false);
  neckZero[0] = prefs.getInt("pan0", 512); neckZero[1] = prefs.getInt("tilt0", 512);
  readPower();
  delay(300);
  jawTo(0);
  Serial.println("realant says hello - sesant_v3_body (ESP32-S2 + MG90S legs, SCS0009 neck, MG90S jaw, home puck)");
}

void loop() {
  pump(Serial, usbLine);
  pump(Serial0, linkLine);
  unsigned long now = millis();

  if (ramp.active) {
    float u = constrain((float)(now - ramp.t0) / (float)ramp.dur, 0.0f, 1.0f);
    float s = 0.5f - 0.5f * cosf(u * PI);
    for (int i = 0; i < 8; i++) { setpoint[i] = ramp.from[i] + (int)lroundf((ramp.to[i] - ramp.from[i]) * s); writeJoint(i); }
    if (u >= 1.0f) {
      ramp.active = false;
      if (ramp.next == 255 && ++curKey < nKeys) { int t[8]; for (int i = 0; i < 8; i++) t[i] = keys[curKey].q[i]; startRamp(t, keys[curKey].ms, 255); }
      else if (ramp.next == ST_DOCKED) settleT0 = now;
      else if (ramp.next != 255) state = ramp.next;
      else nKeys = 0;
    }
  }
  if (state == ST_DOCKING && !ramp.active && settleT0 && now - settleT0 > SETTLE_MS) {
    settleT0 = 0; detachAll(); readPower();
    state = chg_mV > CHG_PRESENT_MV ? ST_CHARGING : ST_DOCK_FAIL;
    Serial.println(state == ST_CHARGING ? "- dock: seated on the puck, charging" : "- dock: no charge voltage on the pads (missed the funnel? puck unpowered?)");
  }

  float dt = (float)(now - lastEst); lastEst = now;
  float maxStep = SERVO_DEG_PER_MS * dt / AX_DEG_PER_UNIT;
  for (int i = 0; i < 8; i++) { float target = attached ? setpoint[i] : estimate[i]; estimate[i] += constrain(target - estimate[i], -maxStep, maxStep); }

  if (now - lastNeckRead >= 100) { lastNeckRead = now; readNeck(); }   // 2 short bus transactions, ~0.3 ms

  if (now - lastMeas >= 20) {
    lastMeas = now;
    readPower();
    if (state == ST_CHARGING && chg_mV < CHG_PRESENT_MV) state = ST_DOCKED;
    else if (state == ST_DOCKED && chg_mV > CHG_PRESENT_MV) state = ST_CHARGING;
    if (vbat_mV > 3000 && vbat_mV < VBAT_LOW_MV && chg_mV < CHG_PRESENT_MV && now - lastLowWarn > 60000) {
      lastLowWarn = now; Serial.println("- battery low: go home (walk over the puck, then send dock)");
    }
    Serial.print("meas "); Serial.print(now); Serial.print("\t");
    for (int i = 0; i < 8; i++) { Serial.print((int)lroundf(estimate[i])); Serial.print("\t"); }
    for (int i = 0; i < 8; i++) { Serial.print(setpoint[i]); Serial.print("\t"); }
    for (int i = 0; i < 4; i++) { Serial.print(0); Serial.print("\t"); }
    for (int i = 0; i < 8; i++) { Serial.print(0); Serial.print("\t"); }
    Serial.print(ESP.getFreeHeap()); Serial.print("\t");
    Serial.print(vbat_mV); Serial.print("\t"); Serial.print(chg_mV); Serial.print("\t"); Serial.print(state); Serial.print("\t");
    Serial.print(neckMeas[0], 1); Serial.print("\t"); Serial.print(neckMeas[1], 1); Serial.print("\t"); Serial.print(jawDeg);
    Serial.println();
  }
}
