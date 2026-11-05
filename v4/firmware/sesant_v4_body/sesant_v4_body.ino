// SesAnt v4: RealAnt-protocol bridge + home puck + StackChan neck + two Jumper-style arms with two-jaw claws
// ---------------------------------------------------------------------------
// Builds on v3 (v3/firmware/sesant_v3_body): legs, battery, dock/undock on the home puck, SCS0009 neck on Serial1,
// CoreS3 command link on Serial0, paw. The v3 mandibles are replaced by two arms (v4/README.md), 3 MG90S each:
//   shoulder yaw (+ = outward)   GPIO 9 left / 7 right    -5 .. +15 deg
//   shoulder pitch (+ = down)    GPIO 12 left / 11 right  -15 .. +70 deg
//   gripper (+ = open)           GPIO 16 left / 15 right   0 .. 35 deg per jaw: the claws grab horizontally, a gear pair
//                                                          swings both jaws sideways together. The gripper servo sits on the
//                                                          forearm behind the shoulder and drives the finger jaw through a
//                                                          wire pushrod: servo angle = GRIP_THETA0 + GRIP_GAIN * jaw (CAD fit)
// Limits come from the CAD sweep (v4/assembly/sesant_v4_arms.json): the arms stay clear of each other, the housing and
// the front legs. Walk with the arms in the carry pose (level): then the front hips keep about +-25 deg; lifting the
// arms crowds the knee servos, so `dock` lifts them only once the robot is about to sit.
// Commands (in addition to v1-v3: attach_servos, detach_servos, reset, standup, sN <pos>, dock, undock, battery,
//           neck <pan> <tilt>, neck_zero, neck_torque, paw):
//   arm_l <yaw> <pitch> <jaw> | arm_r <yaw> <pitch> <jaw> | arms <yaw> <pitch> <jaw>
//   jaw <deg> | grab | release   both grippers
//   carry                        arms level, claws closed (walking pose)
//   reach                        both arms down to the floor in front, claws open, head looking at them (standing)
// meas line: the 29 RealAnt fields + free heap, vbat_mV charge_mV dock_state (v2), pan_deg tilt_deg (v3), then the
//            6 arm setpoints: yaw_l pitch_l jaw_l yaw_r pitch_r jaw_r. ant_server.py reads the first 29.
// Joint convention as v1 for s1..s8 (FL hip, FL knee, BL hip, BL knee, BR hip, BR knee, FR hip, FR knee).

#include <ESP32Servo.h>
#include <Preferences.h>

const int servoPins[8] = {1, 2, 4, 6, 8, 10, 13, 14};
enum SesameCh : uint8_t { R1 = 0, R2 = 1, L1 = 2, L2 = 3, R4 = 4, R3 = 5, L3 = 6, L4 = 7 };
#define VBAT_PIN 3
#define CHG_PIN 5
// arm servo pins: [0] = left, [1] = right
const int ARM_YAW_PIN[2] = {9, 7}, ARM_PITCH_PIN[2] = {12, 11}, ARM_JAW_PIN[2] = {16, 15};
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
const int   PAN_MIN = -120, PAN_MAX = 120, TILT_MIN = -20, TILT_MAX = 60, REACH_TILT = 51;
const float SCS_STEP_PER_DEG = 3.2f;                   // StackChan hal_servo.cpp: 0.3125 deg per step
int   neckZero[2] = {512, 512};
int   neckSign[2] = {+1, -1};                          // tilt: + = look down (check once, flip in neck_zero if needed)
float neckMeas[2] = {-999, -999};
// arms
const int   YAW_MIN = -5, YAW_MAX = 15, PITCH_MIN = -15, PITCH_MAX = 70, JAW_OPEN_DEG = 35;
const int   CARRY_PITCH = 0, DOCK_PITCH = -15, REACH_PITCH = 19;
const float GRIP_THETA0 = 18.4f, GRIP_GAIN = -1.07f;     // pushrod linkage: servo deg at jaw 0, servo deg per jaw deg (v4_arms_report)
// servo direction per joint and side (the left arm is the mirror image): flip a sign if a joint moves the wrong way
int8_t ARM_DIR[3][2] = {{+1, -1}, {+1, -1}, {+1, -1}};     // [yaw, pitch, jaw][left, right]
int8_t ARM_TRIM[3][2] = {{0, 0}, {0, 0}, {0, 0}};

enum DockState : uint8_t { ST_FREE = 0, ST_DOCKING = 1, ST_DOCKED = 2, ST_CHARGING = 3, ST_UNDOCKING = 4, ST_DOCK_FAIL = 5 };

Servo servos[8], armServo[3][2];
Preferences prefs;
HardwareSerial &BUS = Serial1;
bool attached = false;
int setpoint[8] = {512, 224, 512, 224, 512, 224, 512, 224};
float estimate[8] = {512, 224, 512, 224, 512, 224, 512, 224};
String usbLine, linkLine;
unsigned long lastMeas = 0, lastEst = 0, lastLowWarn = 0, lastNeckRead = 0, settleT0 = 0;
uint8_t state = ST_FREE;
int vbat_mV = 0, chg_mV = 0;
int armDeg[3][2] = {{0, 0}, {CARRY_PITCH, CARRY_PITCH}, {0, 0}};

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

// --- arms
void armJoint(int joint, int side, int deg) {
  const int lo[3] = {YAW_MIN, PITCH_MIN, 0}, hi[3] = {YAW_MAX, PITCH_MAX, JAW_OPEN_DEG};
  const int *pins[3] = {ARM_YAW_PIN, ARM_PITCH_PIN, ARM_JAW_PIN};
  armDeg[joint][side] = constrain(deg, lo[joint], hi[joint]);
  Servo &sv = armServo[joint][side];
  if (!sv.attached()) { sv.setPeriodHertz(50); sv.attach(pins[joint][side], 500, 2400); }
  float servoDeg = joint == 2 ? GRIP_THETA0 + GRIP_GAIN * armDeg[joint][side] : armDeg[joint][side];   // gripper goes through the pushrod
  sv.write(constrain(90 + (int)lroundf(ARM_DIR[joint][side] * servoDeg) + ARM_TRIM[joint][side], 0, 180));
}
void armTo(int side, int yaw, int pitch, int jaw) { armJoint(0, side, yaw); armJoint(1, side, pitch); armJoint(2, side, jaw); }
void jawTo(int deg) { armJoint(2, 0, deg); armJoint(2, 1, deg); }
void armsCarry() { for (int s = 0; s < 2; s++) armTo(s, 0, CARRY_PITCH, 0); }

// --- macros
void cmdDock() {
  if (!attached) attachAll();
  int t[8]; poseTargets(KNEE_DOCK_UNITS, t);
  neckTo(0, 0); for (int s = 0; s < 2; s++) armTo(s, 0, DOCK_PITCH, 0);   // arms up, clear of the puck
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
void cmdReach() {
  if (!attached) attachAll();
  int t[8]; poseTargets(KNEE_STAND_UNITS, t);
  for (int s = 0; s < 2; s++) armTo(s, 0, REACH_PITCH, JAW_OPEN_DEG);
  neckTo(0, REACH_TILT, 800);
  state = ST_FREE; startRamp(t, 600, ST_FREE);
  Serial.println("- reach: claws open at the floor in front, head looking at them");
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
  float a = 0, b = 0, c3 = 0;
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
  else if (line.startsWith("arm_l") && sscanf(line.c_str(), "arm_l %f %f %f", &a, &b, &c3) == 3) armTo(0, (int)a, (int)b, (int)c3);
  else if (line.startsWith("arm_r") && sscanf(line.c_str(), "arm_r %f %f %f", &a, &b, &c3) == 3) armTo(1, (int)a, (int)b, (int)c3);
  else if (line.startsWith("arms") && sscanf(line.c_str(), "arms %f %f %f", &a, &b, &c3) == 3) { armTo(0, (int)a, (int)b, (int)c3); armTo(1, (int)a, (int)b, (int)c3); }
  else if (line.startsWith("jaw") && sscanf(line.c_str(), "jaw %f", &a) == 1) jawTo((int)a);
  else if (line.startsWith("carry"))   armsCarry();
  else if (line.startsWith("reach"))   cmdReach();
  else if (line.startsWith("grab"))    jawTo(0);
  else if (line.startsWith("release")) jawTo(JAW_OPEN_DEG);
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
  armsCarry();
  Serial.println("realant says hello - sesant_v4_body (ESP32-S2 + MG90S legs, SCS0009 neck, two 3-servo Jumper arms, home puck)");
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
    Serial.print(neckMeas[0], 1); Serial.print("\t"); Serial.print(neckMeas[1], 1); for (int s = 0; s < 2; s++) for (int j = 0; j < 3; j++) { Serial.print("\t"); Serial.print(armDeg[j][s]); }
    Serial.println();
  }
}
