// SesAnt RealAnt-protocol bridge
// ---------------------------------------------------------------------------
// Runs on the Sesame leg electronics (Lolin ESP32-S2 Mini + 8x MG90S; SSD1306 status OLED optional)
// and speaks the *RealAnt* serial protocol (realant/ant14_cmd_dxl_nano33iot), so the
// unmodified RealAnt Python stack (ant_server.py -> ZeroMQ -> realant-rl) can drive SesAnt.
// In SesAnt the face is the StackChan head (M5Stack CoreS3), which can send these same lines over UART.
//
//   PC -> robot (one line, '\n' terminated, 115200 baud):
//     attach_servos | detach_servos | reset | standup
//     s1 <pos> s2 <pos> ... s8 <pos>        pos in Dynamixel AX-12 units (0..1023 = 0..300 deg, 512 = centre)
//   robot -> PC every 20 ms:
//     meas <ms>\t<8 x position>\t<8 x setpoint>\t<4 x foot>\t<8 x temp>\t<free heap>
//
// MG90S hobby servos have no position feedback. The 8 "position" fields are an OPEN-LOOP
// ESTIMATE (setpoint followed at the MG90S no-load speed, ~0.1 s / 60 deg). Feet and
// temperatures are reported as 0, exactly like the stock RealAnt v1.2 board does for feet.
// Keep that in mind when you train: observations are commanded, not measured, joint angles.
//
// Joint convention (matches RealAnt): odd ids = hips, even ids = knees.
//   hip  512  -> leg points radially outward (SesAnt neutral; mount horns at servo 90 deg like that)
//   knee 224  -> foot vertical (RealAnt "standup")      knee 512 -> foot horizontal, pointing outward
// SesAnt mechanical limits from the collision sweep: hip +-90 deg; knee up to 120 deg OUTWARD and 50 deg INWARD of vertical
// (checks.py reports this as -120..+50 because its sign is + = inward; here + = outward, like RealAnt's 512 = flat outward).
//
// Wiring: identical to Sesame (see sesame-robot/docs/wiring-guide). Servo channel order and pins
// below are copied from sesame-firmware-main.ino (S2 Mini build).

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---- Sesame hardware (S2 Mini build) -------------------------------------------------------
const int servoPins[8] = {1, 2, 4, 6, 8, 10, 13, 14};
enum SesameCh : uint8_t { R1 = 0, R2 = 1, L1 = 2, L2 = 3, R4 = 4, R3 = 5, L3 = 6, L4 = 7 };
#define I2C_SDA 33
#define I2C_SCL 35
#define OLED_I2C_ADDR 0x3C

// ---- RealAnt id (1..8) -> Sesame channel ----------------------------------------------------
// Legs counter-clockwise seen from above, starting front-left: FL, BL, BR, FR.
// (hip, knee) pairs: s1/s2 = FL (L1/L3), s3/s4 = BL (L2/L4), s5/s6 = BR (R2/R4), s7/s8 = FR (R1/R3).
// Re-order here if your realant-rl policy expects a different leg order.
const uint8_t REALANT_TO_CH[8] = {L1, L3, L2, L4, R2, R4, R1, R3};
// +1/-1 per RealAnt id so that "bigger AX value" turns the joint the same way as on RealAnt.
// Mirrored left/right legs on Sesame mount their servos mirrored, hence the sign pattern.
int8_t DIR[8] = {+1, -1, +1, -1, +1, +1, +1, +1};
// Per-joint trim in servo degrees (calibrate: send "reset" + "standup", adjust until legs radial / feet vertical).
int8_t TRIM[8] = {0, 0, 0, 0, 0, 0, 0, 0};

const float AX_DEG_PER_UNIT = 300.0f / 1023.0f;
const int   AX_CENTER = 512;
const int   KNEE_STAND_UNITS = 224;            // RealAnt standup knee value == SesAnt foot vertical
const float SERVO_DEG_PER_MS = 0.6f;           // MG90S ~0.1 s / 60 deg @ 5 V (open-loop estimate)
const int   HIP_LIMIT_DEG = 88;                // keep 2 deg margin to the 90 deg collision-free limit
const int   KNEE_MIN_DEG = -48, KNEE_MAX_DEG = 118;   // - = inward (foot under the body), + = outward; 2 deg margin

Servo servos[8];
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOk = false, attached = false;
int setpoint[8] = {512, 224, 512, 224, 512, 224, 512, 224};
float estimate[8] = {512, 224, 512, 224, 512, 224, 512, 224};
String inputString;
unsigned long lastMeas = 0, lastEst = 0, rxCount = 0;

static inline bool isKnee(int i) { return (i % 2) == 1; }

// AX units -> joint angle (deg, 0 = SesAnt neutral) -> servo command (deg)
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
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(22, 6);  display.print("( o_o )");
  display.setTextSize(1); display.setCursor(0, 34);  display.print("SesAnt  RealAnt-proto");
  display.setCursor(0, 48); display.print(line2);
  display.display();
}

void handleLine(String line) {
  line.trim();
  Serial.print("- recv: "); Serial.println(line);
  rxCount++;
  if (line.startsWith("attach_servos")) { attachAll(); Serial.println("- Succeeded to change torque on"); face("servos ON"); }
  else if (line.startsWith("detach_servos")) { detachAll(); Serial.println("- Succeeded to change torque off"); face("servos OFF (limp)"); }
  else if (line.startsWith("reset"))   { for (int i = 0; i < 8; i++) { setpoint[i] = 512; writeJoint(i); } face("reset: legs flat"); }
  else if (line.startsWith("standup")) { for (int i = 0; i < 8; i++) { setpoint[i] = isKnee(i) ? KNEE_STAND_UNITS : 512; writeJoint(i); } face("standup"); }
  else {
    // "s1 512 s2 224 ..." -> tokenise pairs
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
  Wire.begin(I2C_SDA, I2C_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  face("waiting for host");
  delay(300);
  Serial.println("realant says hello - sesant_realant_bridge (ESP32-S2 + MG90S, open-loop positions)");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    inputString += c;
    if (c == '\n') { handleLine(inputString); inputString = ""; }
    if (inputString.length() > 256) inputString = "";
  }
  unsigned long now = millis();
  // open-loop position estimate (AX units)
  float dt = (float)(now - lastEst); lastEst = now;
  float maxStep = SERVO_DEG_PER_MS * dt / AX_DEG_PER_UNIT;
  for (int i = 0; i < 8; i++) {
    float target = attached ? setpoint[i] : estimate[i];
    estimate[i] += constrain(target - estimate[i], -maxStep, maxStep);
  }
  if (now - lastMeas >= 20) {
    lastMeas = now;
    Serial.print("meas "); Serial.print(now); Serial.print("\t");
    for (int i = 0; i < 8; i++) { Serial.print((int)lroundf(estimate[i])); Serial.print("\t"); }
    for (int i = 0; i < 8; i++) { Serial.print(setpoint[i]); Serial.print("\t"); }
    for (int i = 0; i < 4; i++) { Serial.print(0); Serial.print("\t"); }
    for (int i = 0; i < 8; i++) { Serial.print(0); Serial.print("\t"); }
    Serial.print(ESP.getFreeHeap());
    Serial.println();
  }
}
