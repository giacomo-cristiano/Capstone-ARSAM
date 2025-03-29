#include <PWMServo.h>

PWMServo JointFour;  // Create a servo object
PWMServo JointFive; 
PWMServo JointSix;

int angle6 = 270; // Default starting angles
int angle5 = 90; 
int angle4 = 90 
void setup() {
    // JointFour.attach(4);  // Attach to a PWM-capable pin on Teensy
    // JointFive.attach(5);
    JointSix.attach(6);

    JointSix.write(map(angle6, 0, 270, 0, 180));
}

void loop() {

    delay(1000);
    // JointFour.write(90);  // Move servo to 270 degrees
    // delay(1000);
    // JointFour.write(0);   // Move servo to 0 degrees
    // delay(1000);

    // JointFive.write(270);  // Move servo to 270 degrees
    // delay(1000);
    // JointFive.write(0);   // Move servo to 0 degrees
    // delay(1000);

    JointSix.write(map(angle6, 0, 270, 0, 180));
    Serial.print("Joint 6 set to: "); Serial.println(angle6);
    delay(1000);
    // JointSix.write(0);   // Move servo to 0 degrees
    // delay(1000);
}
