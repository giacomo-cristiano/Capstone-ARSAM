#include <PWMServo.h>

PWMServo myServo;  // Create a servo object

void setup() {
    myServo.attach(4);  // Attach to a PWM-capable pin on Teensy
}

void loop() {
    myServo.write(270);  // Move servo to 270 degrees
    delay(1000);
    myServo.write(0);   // Move servo to 0 degrees
    delay(1000);
}
