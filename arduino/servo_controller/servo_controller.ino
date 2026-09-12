/* Physical 2-DOF starter, Arduino UNO R3. No ANN and no position telemetry here.
 * Read hardware/calibration_checklist.md BEFORE attaching links or enabling ARM.
 * External regulated 5 V servo supply; common GND; signals D9/D10; stop button D2.
 */
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ENABLE_LCD 0
#if ENABLE_LCD
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
hd44780_I2Cexp lcd;
bool lcdReady = false;
#endif

// EXAMPLE calibration for the DS3218 POSITIONAL 270-degree version, not measured
// calibration of your motors. Change these after the single-servo bench test.
const bool CALIBRATION_VERIFIED = false;
const byte SERVO_PINS[2] = {9, 10};
const byte STOP_PIN = 2;  // Optional normally-open button from D2 to GND.
const float NEUTRAL_ANGLE[2] = {45.0f, 0.0f};  // Robot angle at 1500 us.
const float NEUTRAL_US[2] = {1500.0f, 1500.0f};
const float US_PER_DEG[2] = {7.407407f, 7.407407f}; // Signed; measure each axis.
const float LOWER[2] = {0.0f, 0.0f};
const float UPPER[2] = {120.0f, 110.0f};
const int MIN_US[2] = {800, 800};
const int MAX_US[2] = {2350, 2350};
const float HOME[2] = {90.0f, 0.0f};
const float SPEED_DEG_S = 15.0f;
const unsigned long LINK_TIMEOUT_MS = 1500;

Servo motors[2];
bool armed = false, moving = false, dropping = false;
float currentQ[2] = {90.0f, 0.0f}, startQ[2], goalQ[2];
unsigned long moveStart = 0, moveDuration = 0, lastContact = 0, lastUpdate = 0;
unsigned long lastByte = 0, lastDisplay = 0;
unsigned int lastId = 0;
char line[64];
byte used = 0;

float pulseFor(byte axis, float angle) {
  return NEUTRAL_US[axis] + US_PER_DEG[axis] * (angle - NEUTRAL_ANGLE[axis]);
}

bool validAngle(byte axis, float angle) {
  float pulse = pulseFor(axis, angle);
  return isfinite(angle) && angle >= LOWER[axis] && angle <= UPPER[axis]
      && isfinite(pulse) && pulse >= MIN_US[axis] && pulse <= MAX_US[axis];
}

void detachAll() {
  for (byte i = 0; i < 2; ++i) {
    motors[i].detach();
    pinMode(SERVO_PINS[i], OUTPUT);
    digitalWrite(SERVO_PINS[i], LOW);
  }
  armed = moving = false;
  digitalWrite(LED_BUILTIN, LOW);
}

void fault(const __FlashStringHelper *reason) {
  detachAll();
  Serial.print(F("ERR ")); Serial.println(reason);
}

void sendInfo() {
  // INFO version calibrated q1min q1max q2min q2max home1 home2 max_speed
  Serial.print(F("INFO 1 ")); Serial.print(CALIBRATION_VERIFIED ? 1 : 0);
  for (byte i = 0; i < 2; ++i) {
    Serial.print(' '); Serial.print(LOWER[i], 3);
    Serial.print(' '); Serial.print(UPPER[i], 3);
  }
  for (byte i = 0; i < 2; ++i) { Serial.print(' '); Serial.print(HOME[i], 3); }
  Serial.print(' '); Serial.println(SPEED_DEG_S, 3);
}

bool number(char *token, float &out) {
  if (!token || !*token) return false;
  char *end;
  out = strtod(token, &end);
  return *end == '\0' && isfinite(out);
}

void handleLine() {
  char *save;
  char *command = strtok_r(line, " ", &save);
  if (!command) return;
  if (!strcmp(command, "MOVE")) {
    char *idText = strtok_r(NULL, " ", &save);
    float values[2];
    if (!idText || !*idText) { fault(F("PARSE")); return; }
    for (char *p = idText; *p; ++p) {
      if (*p < '0' || *p > '9') { fault(F("PARSE")); return; }
    }
    char *end;
    unsigned long id = strtoul(idText, &end, 10);
    if (*end || !id || id > 65535UL || id <= lastId ||
        !number(strtok_r(NULL, " ", &save), values[0]) ||
        !number(strtok_r(NULL, " ", &save), values[1]) ||
        strtok_r(NULL, " ", &save)) { fault(F("PARSE_OR_ID")); return; }
    if (!armed || moving) { fault(F("NOT_ARMED_OR_BUSY")); return; }
    for (byte i = 0; i < 2; ++i) {
      if (!validAngle(i, values[i])) { fault(F("LIMIT")); return; }
    }
    float largestChange = 0;
    for (byte i = 0; i < 2; ++i) {
      startQ[i] = currentQ[i]; goalQ[i] = values[i];
      largestChange = max(largestChange, fabs(goalQ[i] - startQ[i]));
    }
    // Cubic easing has peak slope 1.5. Both joints arrive together, without
    // independently stepping each axis along an unintended configuration path.
    moveDuration = max(200UL, (unsigned long)ceil(1500.0f * largestChange / SPEED_DEG_S));
    lastId = (unsigned int)id;
    moveStart = lastContact = millis();
    moving = true;
    Serial.print(F("OK ")); Serial.println(lastId);
    return;
  }
  if (strtok_r(NULL, " ", &save)) { fault(F("EXTRA_FIELDS")); return; }
  if (!strcmp(command, "INFO")) { sendInfo(); return; }
  if (!strcmp(command, "PING")) { lastContact = millis(); return; }
  if (!strcmp(command, "STOP")) { detachAll(); Serial.println(F("STOPPED")); return; }
  if (!strcmp(command, "ARM")) {
    if (!CALIBRATION_VERIFIED) { fault(F("CALIBRATION_REQUIRED")); return; }
    if (armed || digitalRead(STOP_PIN) == LOW) { fault(F("ARM_STATE")); return; }
    for (byte i = 0; i < 2; ++i) {
      if (!validAngle(i, LOWER[i]) || !validAngle(i, UPPER[i]) || !validAngle(i, HOME[i]) ||
          !isfinite(US_PER_DEG[i]) || fabs(US_PER_DEG[i]) < 0.01f ||
          MIN_US[i] < 544 || MAX_US[i] > 2400 ||
          !isfinite(SPEED_DEG_S) || SPEED_DEG_S <= 0 || SPEED_DEG_S > 30) {
        fault(F("BAD_CALIBRATION")); return;
      }
    }
    // Operator must have aligned the unpowered arm with HOME first. There are
    // no external encoders: ramping from a fictional starting angle is unsafe.
    // Global Servo objects have their default 544..2400 range before attach;
    // preload the home pulse BEFORE enabling the timer output (no 1500-us jump).
    for (byte i = 0; i < 2; ++i) {
      currentQ[i] = HOME[i];
      motors[i].writeMicroseconds((int)lround(pulseFor(i, HOME[i])));
      motors[i].attach(SERVO_PINS[i], MIN_US[i], MAX_US[i]);
    }
    lastId = 0; armed = true; lastContact = millis();
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println(F("ARMED"));
    return;
  }
  fault(F("COMMAND"));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STOP_PIN, INPUT_PULLUP);
  detachAll();  // Boot and USB reset NEVER arm the motors.
  Serial.begin(115200);
#if ENABLE_LCD
  Wire.begin();
  Wire.setWireTimeout(25000, true);
  lcdReady = (lcd.begin(16, 2) == 0);
#endif
  Serial.println(F("READY 1"));
}

#if ENABLE_LCD
void showLcd(byte row, const char *text) {
  if (!lcdReady) return;
  lcd.setCursor(0, row);
  for (const char *p = text; *p; ++p) {
    if (Wire.getWireTimeoutFlag()) {
      lcdReady = false; Wire.clearWireTimeoutFlag(); return;
    }
    lcd.write(*p);
  }
}
#endif

void loop() {
  unsigned long now = millis();
  if (armed && digitalRead(STOP_PIN) == LOW) fault(F("STOP_BUTTON"));
  if (armed && now - lastContact > LINK_TIMEOUT_MS) fault(F("LINK_TIMEOUT"));
  // Bounded input work keeps the motion/stop checks responsive during a flood.
  for (byte count = 0; count < 64 && Serial.available(); ++count) {
    char c = Serial.read(); lastByte = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!dropping) { line[used] = '\0'; handleLine(); }
      used = 0; dropping = false;
    } else if (!dropping) {
      if (used >= sizeof(line) - 1 || c < 32 || c > 126) {
        used = 0; dropping = true; fault(F("LINE"));
      } else line[used++] = c;
    }
  }
  now = millis(); // Serial handlers can set newer timestamps; avoid unsigned underflow.
  if (used && now - lastByte > 500) {
    used = 0; dropping = true; fault(F("PARTIAL_LINE"));
  }
  if (moving && now - lastUpdate >= 20) {
    lastUpdate = now;
    float t = min(1.0f, (float)(now - moveStart) / moveDuration);
    float eased = t * t * (3.0f - 2.0f * t);
    for (byte i = 0; i < 2; ++i) {
      currentQ[i] = startQ[i] + eased * (goalQ[i] - startQ[i]);
      motors[i].writeMicroseconds((int)lround(pulseFor(i, currentQ[i])));
    }
    if (t >= 1.0f) {
      moving = false;
      Serial.print(F("DONE ")); Serial.print(lastId);
      Serial.print(' '); Serial.print(currentQ[0], 3);
      Serial.print(' '); Serial.println(currentQ[1], 3);
    }
  }
#if ENABLE_LCD
  if (lcdReady && now - lastDisplay >= 250) {
    lastDisplay = now;
    showLcd(0, !armed ? "DISARMED        " : moving ? "MOVING          " : "HOLD / commanded");
    char first[10], second[10], row[17];
    dtostrf(currentQ[0], 5, 1, first); dtostrf(currentQ[1], 5, 1, second);
    snprintf(row, sizeof(row), "%s %s     ", first, second);
    showLcd(1, row);
  }
#endif
}
