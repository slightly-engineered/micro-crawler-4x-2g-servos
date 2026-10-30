/*
 * 4-Servo Quadruped Crawl Gait
 * Arduino Pro Micro
 *
 * Hardware map (as specified):
 *   FL -> pin 16
 *   FR -> pin 8
 *   BL -> pin 14
 *   BR -> pin 7
 *
 * Assumption: each leg has ONE servo (single DOF), sweeping between a
 * PLANT angle (foot down, bearing body weight) and a LIFT angle (foot
 * up, clear of the ground, reaching for the next step). Forward motion
 * comes from a fast LIFT move (unloaded) followed by a slow, weighted
 * PLANT move (drives the body) while the other three legs stay down.
 *
 * Sequence used: (FR + BL) together, then (FL + BR) together
 * Diagonal-pair (trot-style) gait: each diagonal pair moves at the
 * same time — lift then plant — with the follower leg starting a
 * few milliseconds after its pair-lead for a natural, non-robotic
 * feel, while the other diagonal pair stays planted for support.
 *
 * IMPORTANT: plantAngle[]/liftAngle[] values below are placeholders.
 * Opposite-side servos are almost always mounted mirrored, so "lift"
 * is NOT the same raw angle direction on every leg. Use
 * CALIBRATION_MODE below to measure each leg individually before
 * trusting the gait.
 */

#include <Servo.h>

// ---------------- Mode switch ----------------
// 1 = CALIBRATION MODE: center servos for horn mounting, then jog
//     one leg at a time over Serial to find its real PLANT (down)
//     and LIFT (up) angles. Gait does NOT run in this mode.
// 0 = GAIT MODE: runs the walking sequence using the angle arrays
//     below. Only switch to this after calibration is done.
#define CALIBRATION_MODE 0

// ---------------- Pin map ----------------
#define PIN_FL 16
#define PIN_FR 8
#define PIN_BL 14
#define PIN_BR 7

#define CENTER_ANGLE 90 // neutral position for mounting servo horns

// ---------------- Servo objects ----------------
Servo servoFL, servoFR, servoBL, servoBR;

// ---------------- Leg indices ----------------
enum LegID { FL = 0, FR = 1, BL = 2, BR = 3 };
const uint8_t NUM_LEGS = 4;
const char* legName[NUM_LEGS] = { "FL", "FR", "BL", "BR" };

Servo* legServo[NUM_LEGS] = { &servoFL, &servoFR, &servoBL, &servoBR };

// ---------------- Calibration: per-leg angles ----------------
// plantAngle = foot DOWN, planted, bearing body weight
// liftAngle  = foot UP, clear of the ground, reaching for next step
//
// These do NOT follow a pattern across legs (e.g. "lower = up")
// because opposite-side servos are usually mounted mirrored. Each
// pair MUST be measured per leg using CALIBRATION_MODE.
//
// Fill these in AFTER running calibration mode. Values below are
// placeholders only — do not run GAIT MODE until they're measured.
int plantAngle[NUM_LEGS] = { 120, 120, 50, 50 };  // FL, FR, BL, BR — DOWN
int liftAngle[NUM_LEGS]  = { 50, 50, 120, 120 };  // FL, FR, BL, BR — UP

// Tracks each leg's current commanded angle so moves are always
// interpolated from where the leg actually is.
int currentAngle[NUM_LEGS];

// ---------------- Gait timing ----------------
// Per-degree delay controls speed. Larger = slower/smoother.
const uint8_t LIFT_STEP_DELAY_MS  = 12;  // unloaded, still slower for small servos

// Plant (downward push) step delay, per leg. Lower = faster = more
// aggressive push. Front legs run faster/harder on the plant stroke;
// rear legs unchanged. (Hobby servos only expose position via PWM —
// there's no separate torque control — so "more push" in software
// means driving the motion faster, not a literal power increase.)
uint8_t plantStepDelay[NUM_LEGS] = { 15, 15, 25, 25 }; // FL, FR, BL, BR

const uint16_t PAUSE_BETWEEN_LEGS_MS = 150; // settle time, all feet planted

// How far behind the lead leg the trailing leg starts its move, within
// a diagonal pair. Small value = mostly-simultaneous with a natural
// stagger instead of a hard lock-step. 0 = perfectly simultaneous.
const uint16_t PAIR_STAGGER_MS = 40;

// Diagonal-pair walk order: FR leads BL, then FL leads BR. Each pair
// moves together (staggered) while the other diagonal pair stays
// planted for support.
struct LegPair { LegID lead; LegID follow; };
const LegPair gaitPairs[2] = { { FR, BL }, { FL, BR } };

// ---------------- Smooth move ----------------
// Moves a single leg from its current angle to target, one degree at
// a time, with the given per-step delay. This is what gives the
// "smooth/natural" motion instead of a servo snap.
void moveLegSmooth(LegID leg, int targetAngle, uint8_t stepDelayMs) {
  int &angle = currentAngle[leg];
  int step = (targetAngle > angle) ? 1 : -1;

  while (angle != targetAngle) {
    angle += step;
    legServo[leg]->write(angle);
    delay(stepDelayMs);
  }
}

// Moves two legs toward their targets concurrently (non-blocking,
// millis()-timed), with the follow leg starting staggerMs after the
// lead leg. Each leg can use its own step delay (e.g. front legs
// faster than rear on the plant stroke). Returns once both legs have
// reached their targets.
void movePairSmooth(LegID lead, LegID follow, int targetLead, int targetFollow,
                     uint8_t stepDelayLead, uint8_t stepDelayFollow, uint16_t staggerMs) {
  int &angleLead = currentAngle[lead];
  int &angleFollow = currentAngle[follow];
  int stepLead = (targetLead > angleLead) ? 1 : -1;
  int stepFollow = (targetFollow > angleFollow) ? 1 : -1;

  bool leadDone = (angleLead == targetLead);
  bool followDone = (angleFollow == targetFollow);
  bool followStarted = false;

  unsigned long now = millis();
  unsigned long lastStepLead = now;
  unsigned long followStartAt = now + staggerMs;
  unsigned long lastStepFollow = 0;

  while (!leadDone || !followDone) {
    now = millis();

    if (!leadDone && now - lastStepLead >= stepDelayLead) {
      angleLead += stepLead;
      legServo[lead]->write(angleLead);
      lastStepLead = now;
      if (angleLead == targetLead) leadDone = true;
    }

    if (!followDone) {
      if (!followStarted) {
        if (now >= followStartAt) {
          followStarted = true;
          lastStepFollow = now;
        }
      } else if (now - lastStepFollow >= stepDelayFollow) {
        angleFollow += stepFollow;
        legServo[follow]->write(angleFollow);
        lastStepFollow = now;
        if (angleFollow == targetFollow) followDone = true;
      }
    }
  }
}

// One full step cycle for a single leg: lift, then plant (the power
// stroke that actually moves the body). Used by calibration-adjacent
// single-leg helpers; the gait itself uses stepPair() below.
void stepLeg(LegID leg) {
  moveLegSmooth(leg, liftAngle[leg], LIFT_STEP_DELAY_MS);     // up
  moveLegSmooth(leg, plantAngle[leg], plantStepDelay[leg]);   // down / push
  delay(PAUSE_BETWEEN_LEGS_MS);
}

// One full step cycle for a diagonal pair: both legs lift together
// (follow slightly staggered), then both plant together (staggered,
// each leg using its own plant speed), while the other diagonal pair
// stays planted for support.
void stepPair(const LegPair& pair) {
  movePairSmooth(pair.lead, pair.follow, liftAngle[pair.lead], liftAngle[pair.follow],
                 LIFT_STEP_DELAY_MS, LIFT_STEP_DELAY_MS, PAIR_STAGGER_MS);
  movePairSmooth(pair.lead, pair.follow, plantAngle[pair.lead], plantAngle[pair.follow],
                 plantStepDelay[pair.lead], plantStepDelay[pair.follow], PAIR_STAGGER_MS);
  delay(PAUSE_BETWEEN_LEGS_MS);
}

// Sends all four servos straight to CENTER_ANGLE. Use this before
// mounting servo horns so they're all at a known neutral position,
// and any time you want to reset to neutral.
void centerAllServos() {
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    currentAngle[i] = CENTER_ANGLE;
    legServo[i]->write(CENTER_ANGLE);
  }
  Serial.print(F("All legs centered to "));
  Serial.println(CENTER_ANGLE);
}

#if CALIBRATION_MODE

// ---------------- Calibration mode ----------------
// Serial commands (115200 baud, newline-terminated):
//   c                -> center all 4 servos (mount horns now)
//   <LEG> <ANGLE>     -> move one leg directly to ANGLE, e.g. "FR 105"
//                        LEG is one of: FL FR BL BR (case-insensitive)
//   ?                 -> print current angle of all 4 legs
//
// Workflow: send "c", mount all horns square/neutral. Then for each
// leg, send angles like "FL 70", "FL 110" etc. until you find the
// angle that plants the foot flat on the ground (write it down as
// plantAngle) and the angle that lifts the foot clear (write it down
// as liftAngle). Repeat for all 4 legs, plug the 8 numbers into the
// arrays above, set CALIBRATION_MODE to 0, re-upload.

String serialLine;

int legIndexFromName(const String& name) {
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    if (name.equalsIgnoreCase(legName[i])) return i;
  }
  return -1;
}

void printAllAngles() {
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    Serial.print(legName[i]);
    Serial.print(F("="));
    Serial.print(currentAngle[i]);
    Serial.print(F("  "));
  }
  Serial.println();
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("c")) {
    centerAllServos();
    return;
  }
  if (line == "?") {
    printAllAngles();
    return;
  }

  int spaceIdx = line.indexOf(' ');
  if (spaceIdx <= 0) {
    Serial.println(F("Unrecognized. Use: c | ? | <LEG> <ANGLE>"));
    return;
  }

  String legStr = line.substring(0, spaceIdx);
  String angleStr = line.substring(spaceIdx + 1);
  angleStr.trim();

  int idx = legIndexFromName(legStr);
  if (idx < 0) {
    Serial.println(F("Unknown leg. Use FL, FR, BL, or BR."));
    return;
  }

  int angle = angleStr.toInt();
  angle = constrain(angle, 0, 180);

  currentAngle[idx] = angle;
  legServo[idx]->write(angle);

  Serial.print(legName[idx]);
  Serial.print(F(" -> "));
  Serial.println(angle);
}

void setup() {
  Serial.begin(115200);
  servoFL.attach(PIN_FL);
  servoFR.attach(PIN_FR);
  servoBL.attach(PIN_BL);
  servoBR.attach(PIN_BR);

  centerAllServos();

  Serial.println(F("=== CALIBRATION MODE ==="));
  Serial.println(F("Servos centered. Mount horns now if needed."));
  Serial.println(F("Commands: c (center all) | ? (show angles) | <LEG> <ANGLE> e.g. FR 105"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleSerialLine(serialLine);
      serialLine = "";
    } else if (c != '\r') {
      serialLine += c;
    }
  }
}

#else

// ---------------- Gait mode ----------------
void setup() {
  servoFL.attach(PIN_FL);
  servoFR.attach(PIN_FR);
  servoBL.attach(PIN_BL);
  servoBR.attach(PIN_BR);

  // Start centered (no snap on power-up), then ease into plant pose.
  centerAllServos();
  delay(300);
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    moveLegSmooth((LegID)i, plantAngle[i], plantStepDelay[i]);
  }
  delay(500);
}

void loop() {
  for (uint8_t i = 0; i < 2; i++) {
    stepPair(gaitPairs[i]);
  }
}

#endif

/*
 * CALIBRATION WORKFLOW:
 *
 * 1. Set CALIBRATION_MODE to 1 (default), upload, open Serial Monitor
 *    at 115200 baud, line ending "Newline".
 * 2. Send "c" to center all servos — mount/adjust horns now so each
 *    leg sits in a sane neutral position at 90 degrees.
 * 3. Power the robot so its feet are on the ground (or supported).
 *    For each leg, send "<LEG> <ANGLE>" (e.g. "FL 70", "FL 110", ...)
 *    and watch the foot. Find and record:
 *      - plantAngle: foot flat/planted, bearing body weight
 *      - liftAngle:  foot lifted clear, no drag or collision with
 *                     neighboring legs
 *    Do this independently for FL, FR, BL, BR — do not assume any
 *    two legs share values, even diagonal or "matching" ones.
 * 4. Plug all 8 measured values into plantAngle[] / liftAngle[] at
 *    the top of the file, in FL, FR, BL, BR order.
 * 5. Set CALIBRATION_MODE to 0, re-upload. This runs the gait.
 * 6. If a leg's motion order still looks reversed (plant before lift
 *    instead of lift before plant), you swapped which measured value
 *    goes in which array — swap that leg's two numbers.
 * 7. Run at the default plantStepDelay values (15ms front / 25ms rear)
 *    no leg collisions, then reduce delay to speed up.
 * 8. If the robot doesn't translate (legs move but body doesn't
 *    advance), the lift-to-plant travel range is too small, or the
 *    feet lack directional grip during the plant stroke — that's a
 *    mechanical, not firmware, fix.
 */
