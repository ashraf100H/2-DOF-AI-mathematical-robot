/* ONE loose servo only, horn/link removed. D9 signal; external regulated 5 V and common
 * ground. Serial Monitor: 115200 baud. N = neutral; +/- = 20 us; X = detach.
 * Starts detached. Initial pulse window is deliberately small. Extend it only
 * after checking the exact variant and mechanical clearance; never seek a hard
 * stop by driving into it. This test is NOT a two-joint robot controller.
 */
#include <Servo.h>
Servo motor;
const int MIN_TEST_US = 1100;
const int MAX_TEST_US = 1900;
int pulse = 1500;
void setup() {
  pinMode(9, OUTPUT); digitalWrite(9, LOW);
  Serial.begin(115200);
  Serial.println(F("One loose servo: N neutral, + or - 20us, X detach."));
}
void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'X' || c == 'x') {
    motor.detach(); digitalWrite(9, LOW); Serial.println(F("Detached"));
  } else if (c == 'N' || c == 'n') {
    pulse = 1500;
    motor.writeMicroseconds(pulse);
    motor.attach(9, 544, 2400);
    Serial.println(pulse);
  } else if ((c == '+' || c == '-') && motor.attached()) {
    int next = pulse + (c == '+' ? 20 : -20);
    if (next >= MIN_TEST_US && next <= MAX_TEST_US) {
      pulse = next; motor.writeMicroseconds(pulse); Serial.println(pulse);
    } else Serial.println(F("Test window limit"));
  }
}
