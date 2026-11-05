// SesAnt v2: RealAnt-protocol bridge + home puck (charging) / battery extension
// ---------------------------------------------------------------------------
// Same hardware and protocol as v1 (firmware/sesant_realant_bridge), plus:
//   * battery monitor  : 2S pack via 100k/27k divider -> GPIO 3 (ADC1)
//   * puck detect      : charge-pad voltage via 100k/27k divider -> GPIO 5 (ADC1), after the input Schottky
//   * new commands     : dock | undock | battery
//   * meas line        : the 29 RealAnt fields + free heap, then 3 extra fields appended:
//                        vbat_mV  charge_mV  dock_state     (ant_server.py reads the first 29 and ignores the rest)
//
// dock    : robot must stand over the home puck (belly cone within ~12 mm of the funnel axis, any heading; the CoreS3
//           homes on the AprilTag flag). Knees open smoothly to the seat pose (feet slide outward on the floor), the
//           belly cone drops into the puck's 30-degree funnel, then the servos go limp: the cone slides down the funnel
//           walls to the centre and the spring pins meet the disc + ring pads. The seat doubles as the RL reset pose.
// undock  : servos on at the seat pose, knees ramp back to standup; the robot can walk off.
// battery : prints "- battery <vbat_mV> <charge_mV> <state>".
//
// Seat pose comes from cad/scripts/v2_puck.py (v2/assembly/sesant_v2_puck.json: seat_knee_realant_units).
// Joint convention as v1: odd ids hips, even ids knees; knee 224 = foot vertical, + = foot outward.

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int servoPins[8] = {1, 2, 4, 6, 8, 10, 13, 14};
enum SesameCh : uint8_t { R1 = 0, R2 = 1, L1 = 2, L2 = 3, R4 = 4, R3 = 5, L3 = 6, L4 = 7 };
#define I2C_SDA 33
#define I2C_SCL 35
#define OLED_I2C_ADDR 0x3C
#define VBAT_PIN 3
#define CHG_PIN 5
const float DIVIDER = (100.0f + 27.0f) / 27.0f;     // 8.4 V -> 1.79 V at the pin

const uint8_t REALANT_TO_CH[8] = {L1, L3, L2, L4, R2, R4, R1, R3};
int8_t DIR[8] = {+1, -1, +1, -1, +1, +1, +1, +1};
int8_t TRIM[8] = {0, 0, 0, 0, 0, 0, 0, 0};

const float AX_DEG_PER_UNIT = 300.0f / 1023.0f;
const int   AX_CENTER = 512;
const int   KNEE_STAND_UNITS = 224;
const int   KNEE_DOCK_UNITS = 396 + 7;           // CAD seat pose (50.6 deg outward) + 2 deg so the cone settles in the funnel
const float SERVO_DEG_PER_MS = 0.6f;
const int   HIP_LIMIT_DEG = 88;
const int   KNEE_MIN_DEG = -48, KNEE_MAX_DEG = 118;   // - = inward, + = outward (collision sweep: 50 in / 120 out)
const unsigned long DOCK_RAMP_MS = 2000, SETTLE_MS = 400;
const int   CHG_PRESENT_MV = 6000;               // pads see the puck charger (8.4 V CV minus the Schottky)
const int   VBAT_LOW_MV = 6900;                  // 2S ~3.45 V/cell under load: time to go home

enum DockState : uint8_t { ST_FREE = 0, ST_DOCKING = 1, ST_DOCKED = 2, ST_CHARGING = 3, ST_UNDOCKING = 4, ST_DOCK_FAIL = 5 };

Servo servos[8];
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOk = false, attached = false;
int setpoint[8] = {512, 224, 512, 224, 512, 224, 512, 224};
float estimate[8] = {512, 224, 512, 224, 512, 224, 512, 224};
String inputString;
unsigned long lastMeas = 0, lastEst = 0, lastLowWarn = 0;
uint8_t state = ST_FREE;
int vbat_mV = 0, chg_mV = 0;

// non-blocking joint ramp (cosine profile) so meas keeps streaming while docking
struct Ramp { bool active; int from[8], to[8]; unsigned long t0, dur; uint8_t next; } ramp = {false};

static inline bool isKnee(int i) { return (i % 2) == 1; }

int jointDeg(int i, int ax) {
  float a = isKnee(i) ? (ax - KNEE_STAND_UNITS) * AX_DEG_PER_UNIT : (ax - AX_CENTER) * AX_DEG_PER_UNIT;
  if (isKnee(i)) a = constrain(a, (float)KNEE_MIN_DEG, (float)KNEE_MAX_DEG);
  else a = constrain(a, (float)-HIP_LIMIT_DEG, (float)HIP_LIMIT_DEG);
  return (int)lroundf(a);
}

void writeJoint(int i) {
  if (!attached) return;
  int cmd = constrain(90 + DIR[i] * jointDeg(i, setpoint[i]) + TRIM[i], 0, 180);
  servos[REALANT_TO_CH[i]].write(cmd);
}

void attachAll() {
  for (int i = 0; i < 8; i++) { servos[i].setPeriodHertz(50); servos[i].attach(servoPins[i], 732, 2929); }
  attached = true;
  for (int i = 0; i < 8; i++) writeJoint(i);
}
void detachAll() { for (int i = 0; i < 8; i++) servos[i].detach(); attached = false; }

void face(const char *line2) {
  if (!oledOk) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0, 8); display.print("SesAnt v2  puck");
  display.setCursor(0, 28); display.printf("bat %d mV", vbat_mV);
  display.setCursor(0, 48); display.print(line2);
  display.display();
}

void startRamp(const int target[8], unsigned long dur, uint8_t next) {
  for (int i = 0; i < 8; i++) { ramp.from[i] = setpoint[i]; ramp.to[i] = target[i]; }
  ramp.t0 = millis(); ramp.dur = dur; ramp.next = next; ramp.active = true;
}

void poseTargets(int kneeUnits, int out[8]) { for (int i = 0; i < 8; i++) out[i] = isKnee(i) ? kneeUnits : AX_CENTER; }

void readPower() {
  vbat_mV = (int)(analogReadMilliVolts(VBAT_PIN) * DIVIDER);
  chg_mV = (int)(analogReadMilliVolts(CHG_PIN) * DIVIDER);
}

void cmdDock() {
  if (!attached) attachAll();
  int t[8]; poseTargets(KNEE_DOCK_UNITS, t);
  state = ST_DOCKING; startRamp(t, DOCK_RAMP_MS, ST_DOCKED);
  Serial.println("- dock: sitting down on the puck"); face("docking...");
}

void cmdUndock() {
  int t[8];
  if (!attached) {   // after docking the servos are limp at the seat pose: start from there, not from a stale setpoint
    poseTargets(KNEE_DOCK_UNITS, setpoint);
    for (int i = 0; i < 8; i++) estimate[i] = setpoint[i];
    attachAll();
  }
  poseTargets(KNEE_STAND_UNITS, t);
  state = ST_UNDOCKING; startRamp(t, DOCK_RAMP_MS, ST_FREE);
  Serial.println("- undock: standing up"); face("undocking...");
}

void handleLine(String line) {
  line.trim();
  Serial.print("- recv: "); Serial.println(line);
  if (line.startsWith("attach_servos")) { attachAll(); Serial.println("- Succeeded to change torque on"); }
  else if (line.startsWith("detach_servos")) { ramp.active = false; detachAll(); Serial.println("- Succeeded to change torque off"); }
  else if (line.startsWith("reset"))   { ramp.active = false; for (int i = 0; i < 8; i++) { setpoint[i] = 512; writeJoint(i); } }
  else if (line.startsWith("standup")) { ramp.active = false; for (int i = 0; i < 8; i++) { setpoint[i] = isKnee(i) ? KNEE_STAND_UNITS : 512; writeJoint(i); } }
  else if (line.startsWith("undock"))  cmdUndock();
  else if (line.startsWith("dock"))    cmdDock();
  else if (line.startsWith("battery")) { readPower(); Serial.printf("- battery %d %d %d\n", vbat_mV, chg_mV, state); }
  else {
    ramp.active = false;
    if (state != ST_FREE && attached) state = ST_FREE;
    char buf[260]; line.toCharArray(buf, sizeof(buf));
    char *save = nullptr; char *tok = strtok_r(buf, " \t", &save);
    while (tok) {
      char *val = strtok_r(nullptr, " \t", &save);
      if (!val) break;
      if (tok[0] == 's') {
        int s = atoi(tok + 1);
        if (s >= 1 && s <= 8) { setpoint[s - 1] = constrain(atoi(val), 0, 1023); writeJoint(s - 1); }
      }
      tok = strtok_r(nullptr, " \t", &save);
    }
  }
}

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  analogSetPinAttenuation(CHG_PIN, ADC_11db);
  Wire.begin(I2C_SDA, I2C_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  readPower(); face("waiting for host");
  delay(300);
  Serial.println("realant says hello - sesant_v2_puck (ESP32-S2 + MG90S, open-loop positions, home puck + battery)");
}

unsigned long settleT0 = 0;

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    inputString += c;
    if (c == '\n') { handleLine(inputString); inputString = ""; }
    if (inputString.length() > 256) inputString = "";
  }
  unsigned long now = millis();

  if (ramp.active) {
    float u = constrain((float)(now - ramp.t0) / (float)ramp.dur, 0.0f, 1.0f);
    float s = 0.5f - 0.5f * cosf(u * PI);
    for (int i = 0; i < 8; i++) { setpoint[i] = ramp.from[i] + (int)lroundf((ramp.to[i] - ramp.from[i]) * s); writeJoint(i); }
    if (u >= 1.0f) {
      ramp.active = false;
      if (ramp.next == ST_DOCKED) settleT0 = now;        // hold briefly, then go limp: the cone centres itself in the funnel
      else { state = ramp.next; face(state == ST_FREE ? "free" : "?"); }
    }
  }
  if (state == ST_DOCKING && !ramp.active && settleT0 && now - settleT0 > SETTLE_MS) {
    settleT0 = 0; detachAll(); readPower();
    state = chg_mV > CHG_PRESENT_MV ? ST_CHARGING : ST_DOCK_FAIL;
    Serial.println(state == ST_CHARGING ? "- dock: seated on the puck, charging" : "- dock: no charge voltage on the pads (missed the funnel? puck unpowered?)");
    face(state == ST_CHARGING ? "charging" : "dock: no contact");
  }

  // open-loop position estimate (AX units)
  float dt = (float)(now - lastEst); lastEst = now;
  float maxStep = SERVO_DEG_PER_MS * dt / AX_DEG_PER_UNIT;
  for (int i = 0; i < 8; i++) {
    float target = attached ? setpoint[i] : estimate[i];
    estimate[i] += constrain(target - estimate[i], -maxStep, maxStep);
  }

  if (now - lastMeas >= 20) {
    lastMeas = now;
    readPower();
    if (state == ST_CHARGING && chg_mV < CHG_PRESENT_MV) state = ST_DOCKED;          // dock power lost / charge done & cut off
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
    Serial.print(vbat_mV); Serial.print("\t"); Serial.print(chg_mV); Serial.print("\t"); Serial.print(state);
    Serial.println();
  }
}
