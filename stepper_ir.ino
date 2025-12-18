/*
  Adaptive Speed Control (Stepper + Ultrasonic + IR + LCD + LED)
  Driver: ULN2003 (IN1..IN4), Motor: 28BYJ-48 (typical)
  LCD: 16x2 parallel (no I2C)

  Pin map:
    Stepper IN1..IN4 -> D8, D9, D10, D11
    Ultrasonic TRIG  -> A0
    Ultrasonic ECHO  -> A1
    IR receiver OUT  -> D7
    LED alert        -> D13
    LCD RS->D12, E->D6, D4->D5, D5->D4, D6->D3, D7->D2, RW->GND, VSS->GND, VDD->5V, VO via pot
*/

#include <Stepper.h>
#include <IRremote.h>
#include <LiquidCrystal.h>

// ---------------- Pins ----------------
#define STP_IN1 8
#define STP_IN2 9
#define STP_IN3 10
#define STP_IN4 11

#define US_TRIG A0
#define US_ECHO A1

#define IR_RX   7
#define LED_PIN A2

// LCD: LiquidCrystal(rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(12, 6, 5, 4, 3, 2);

// ---------------- Stepper ----------------
// 28BYJ-48 via ULN2003, 4-step sequence
const int STEPS_PER_REV = 2048;
Stepper stepper(STEPS_PER_REV, STP_IN1, STP_IN2, STP_IN3, STP_IN4);

// ---------------- Tunables ----------------
const float RPM_L1 = 6.0f;   // level 1
const float RPM_L9 = 15.0f;  // level 9

// Safety distance model (cm)
const int BASE_SAFE_CM  = 10;
const int CM_PER_LEVEL  = 5;
const int MIN_CM        = 2;
const int HYSTERESIS_CM = 3;

// Ultrasonic cadence
const unsigned long ECHO_TIMEOUT_US  = 25000UL; // ~4.3 m
const unsigned long PING_INTERVAL_MS = 60;

// LCD refresh
const unsigned long LCD_REFRESH_MS   = 100;

// IR diagnostic (set to 1 to learn your remote codes)
#define IR_DIAGNOSTIC 0

// ---------------- State ----------------
int commandedLevel = 0; // 0..9 from IR
int effectiveLevel = 0; // 0..9 after regulation
bool reducing = false;

unsigned long lastPing = 0;
unsigned long lastLcd  = 0;
long distanceCm = -1;

// --- LED stability (no flicker) ---
const int LED_ON_CM  = 20;   // turn lamp ON when distance < 20 cm
const int LED_OFF_CM = 25;   // turn lamp OFF when distance > 25 cm
bool ledState = false;       // current LED state

// ------------- Helpers -------------
float levelToRpm(int lvl) {
  if (lvl <= 0) return 0.0f;
  return RPM_L1 + (RPM_L9 - RPM_L1) * (float)(lvl - 1) / 8.0f;
}
int computeSafeCm(int lvl) {
  if (lvl <= 0) return BASE_SAFE_CM;
  return BASE_SAFE_CM + CM_PER_LEVEL * lvl;
}

// Map your IR remote commands to digits here.
int levelFromIrCommand(uint8_t cmd) {
  switch (cmd) {
    case 0x16: return 0;
    case 0x0C: return 1;
    case 0x18: return 2;
    case 0x5E: return 3;
    case 0x08: return 4;
    case 0x1C: return 5;
    case 0x5A: return 6;
    case 0x42: return 7;
    case 0x52: return 8;
    case 0x4A: return 9;
    default:   return -1;
  }
}

long readUltrasonicCm() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long duration = pulseIn(US_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1; // no echo
  long cm = (long)(duration / 58UL); // ~58 us per cm
  if (cm < MIN_CM) cm = MIN_CM;
  return cm;
}

void applyLevel(int lvl) {
  effectiveLevel = constrain(lvl, 0, 9);
  float rpm = levelToRpm(effectiveLevel);
  stepper.setSpeed(rpm); // RPM
}

void updateLcd() {
  lcd.setCursor(0, 0);
  char line1[17];

  // Show 0 when no valid reading yet (you can change this to --- if you like)
  snprintf(line1, sizeof(line1), "L:%d E:%d D:%3ldcm",
           commandedLevel, effectiveLevel, (distanceCm < 0 ? 0 : distanceCm));
  lcd.print(line1);
  for (int i = strlen(line1); i < 16; i++) lcd.print(' ');

  lcd.setCursor(0, 1);

  // obstacle message only when distance < 20 cm
  if (distanceCm >= 0 && distanceCm < 20) {
    const char* msg = "Obstacle Detected";
    lcd.print(msg);
    for (int i = strlen(msg); i < 16; i++) lcd.print(' ');
  } else {
    const char* msg = "MODE: NORMAL   ";
    lcd.print(msg);
  }
}

void setup() {
  Serial.begin(9600);

  // Pins
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // IR
  IrReceiver.begin(IR_RX, ENABLE_LED_FEEDBACK);
  Serial.println(F("IR ready. Press digits 0..9 (0=stop)."));
  if (IR_DIAGNOSTIC) Serial.println(F("Diagnostic: printing received IR command hex."));

  // LCD init
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Adaptive Speed");
  lcd.setCursor(0, 1); lcd.print("System Ready");
  delay(800);
  lcd.clear();

  commandedLevel = 0;
  applyLevel(0);
  reducing = false;
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // ---- IR handling ----
  if (IrReceiver.decode()) {
    auto &d = IrReceiver.decodedIRData;
    if (!(d.flags & IRDATA_FLAGS_IS_REPEAT)) {
#if IR_DIAGNOSTIC
      Serial.print(F("IR cmd=0x")); Serial.println(d.command, HEX);
#else
      int lvl = levelFromIrCommand(d.command);
      if (lvl >= 0 && lvl <= 9) {
        commandedLevel = lvl;
        Serial.print(F("Set level to ")); Serial.println(commandedLevel);
      }
#endif
    }
    IrReceiver.resume();
  }

  // ---- Ultrasonic & regulation ----
  unsigned long now = millis();
  if (now - lastPing >= PING_INTERVAL_MS) {
    lastPing = now;
    distanceCm = readUltrasonicCm();
    int safeCm = computeSafeCm(commandedLevel);

    if (distanceCm < 0) {
      // No echo -> assume clear path (use commanded)
      reducing = false;
      applyLevel(commandedLevel);
    } else if (distanceCm < safeCm) {
      // Reduce proportionally: MIN_CM..safe -> 0..commandedLevel
      long clamped = constrain(distanceCm, MIN_CM, safeCm);
      int reduced = (commandedLevel == 0) ? 0 : map(clamped, MIN_CM, safeCm, 0, commandedLevel);
      reducing = (reduced < commandedLevel);
      applyLevel(reduced);
    } else if (distanceCm > safeCm + HYSTERESIS_CM) {
      reducing = false;
      applyLevel(commandedLevel);
    }

    // --- LED control with hysteresis based only on distance ---
    if (distanceCm >= 0) {
      if (!ledState && distanceCm < LED_ON_CM) {
        ledState = true;          // turn ON when first below 20 cm
      }
      if (ledState && distanceCm > LED_OFF_CM) {
        ledState = false;         // turn OFF only when clearly beyond 25 cm
      }
    }
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  // ---- LCD refresh ----
  if (now - lastLcd >= LCD_REFRESH_MS) {
    lastLcd = now;
    updateLcd();
  }

  // ---- Drive stepper continuously (forward) ----
  if (effectiveLevel > 0) {
    stepper.step(1); // small blocking step; repeat for continuous rotation
  }
  // For reverse direction (optional): stepper.step(-1);
}
