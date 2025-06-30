#include <Servo.h>

Servo servo_motor;
Servo ESC;

void setup() {
  ESC.attach(9);
  servo_motor.attach(10);
  Serial.begin(115200);
  while(!Serial){}
}

void loop() {
  if (Serial.available() >= 3) {
    uint8_t type = Serial.read();
    uint8_t low = Serial.read();
    uint8_t high = Serial.read();
    uint16_t pwm_value = (high << 8) | low;

    
    if (type == 0x01) {
        servo_motor.writeMicroseconds(pwm_value);
    } else if (type == 0x02) {
        ESC.writeMicroseconds(pwm_value);
      }
    }
 }
