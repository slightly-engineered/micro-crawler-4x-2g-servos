/* * 4-Servo Quadruped Crawl Gait (LANL Edition ScoutWalker 2.2)
 * Arduino Pro Micro * 
 * Hardware map (as specified): 
 * FL -> pin 16 
 * FR -> pin 8 
 * BL -> pin 14 
 * BR -> pin 7 
 * 
 * LANL GAIT ENGINE:
 * Implements Mark Tilden's phase-opposition dual-oscillator logic. 
 * Diagonal leg pairs move simultaneously in inverted states: while one 
 * diagonal pair drives the Power Stroke (down/plant), the other diagonal 
 * pair instantly shifts to the Recovery Stroke (up/lift).
 */ 

#include <Servo.h> 

// ---------------- Mode switch ---------------- 
// 1 = CALIBRATION MODE: center servos for horn mounting, then jog 
// one leg at a time over Serial to find its real PLANT (down) 
// and LIFT (up) angles. Gait does NOT run in this mode. 
// 0 = GAIT MODE: runs the walking sequence using the angle arrays 
// below. Only switch to this after calibration is done. 
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
// liftAngle = foot UP, clear of the ground, reaching for next step 
// 
// Fill these in AFTER running calibration mode. Values below are 
// Gently reduce the FR limits to prevent it from over-lifting or pushing too low
int plantAngle[NUM_LEGS] = { 120, 105, 50, 50 }; // Narrowed FR down to 105
int liftAngle[NUM_LEGS]  = { 50,  75, 120, 120 }; // Narrowed FR up to 75

// Tracks each leg's current commanded angle so moves are always 
// interpolated from where the leg actually is. 
int currentAngle[NUM_LEGS]; 

// ---------------- LANL Gait timing ---------------- 
// Controls the simultaneous transition speed of all legs (lower = faster)
const uint8_t LANL_STEP_DELAY_MS = 15; 
// Time spent holding the stance at the peak of each stroke cycle
const uint16_t LANL_PHASE_HOLD_MS = 200; 

// ---------------- LANL Concurrent Phase Transition Engine ---------------- 
// This replicates the analog bicore network by stepping all four legs concurrently 
// toward their targets, mimicking true physical nervous system behavior.
void executeLANLPhase(int targetFL, int targetFR, int targetBL, int targetBR) {
  int targets[NUM_LEGS] = { targetFL, targetFR, targetBL, targetBR };
  bool allDone = false;

  while (!allDone) {
    allDone = true;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
      if (currentAngle[i] != targets[i]) {
        currentAngle[i] += (targets[i] > currentAngle[i]) ? 1 : -1;
        legServo[i]->write(currentAngle[i]);
        allDone = false;
      }
    }
    delay(LANL_STEP_DELAY_MS); // Synchronization clock cycle
  }
}

// Sends all four servos straight to CENTER_ANGLE. 
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
    Serial.print(F(" ")); 
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
// ---------------- LANL Active Gait mode ---------------- 
void setup() { 
  servoFL.attach(PIN_FL); 
  servoFR.attach(PIN_FR); 
  servoBL.attach(PIN_BL); 
  servoBR.attach(PIN_BR); 
  
  centerAllServos(); 
  delay(300); 
  
  // Establish an initial split configuration before beginning sequence
  for (uint8_t i = 0; i < NUM_LEGS; i++) { 
    currentAngle[i] = CENTER_ANGLE;
  } 
  delay(500); 
} 

void loop() { 
  // PHASE A: Pair 1 (FR + BL) Plants DOWN while Pair 2 (FL + BR) Lifts UP
  executeLANLPhase(liftAngle[FL], plantAngle[FR], plantAngle[BL], liftAngle[BR]); 
  delay(LANL_PHASE_HOLD_MS); 

  // PHASE B: Pair 1 (FR + BL) Lifts UP while Pair 2 (FL + BR) Plants DOWN
  executeLANLPhase(plantAngle[FL], liftAngle[FR], liftAngle[BL], plantAngle[BR]); 
  delay(LANL_PHASE_HOLD_MS); 
} 
#endif
