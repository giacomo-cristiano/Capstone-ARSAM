#include <PWMServo.h>

PWMServo JointFour;  // Create servo objects
PWMServo JointFive; 
PWMServo JointSix;

int angle4 = 65; // Default starting angles
int angle5 = 90;
int angle6 = 0;

void setup() {
    Serial.begin(9600); // Start serial communication
    JointFour.attach(4);  // Attach servos to their respective PWM-capable pins on Teensy
    JointFive.attach(5);
    JointSix.attach(6);

    // Move servos to default position
    JointFour.write(map(angle4, 0, 270, 0, 180));
    JointFive.write(map(angle5, 0, 270, 0, 180));
    JointSix.write(map(angle6, 0, 270, 0, 180));
}

void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();  // Clear any leftover characters in the serial buffer
    }
}

void loop() {
    int inputAngle;

    Serial.println("Enter angle for Joint 4 (0 to 270, or -1 to skip): ");
    while (Serial.available() == 0); // Wait for user input
    inputAngle = Serial.parseInt();
    clearSerialBuffer();
    if (inputAngle != -1) { // Only update if not skipping
        angle4 = inputAngle;
        JointFour.write(map(angle4, 0, 270, 0, 180));
    }
    Serial.print("Joint 4 set to: "); Serial.println(angle4);

    Serial.println("Enter angle for Joint 5 (0 to 270, or -1 to skip, Reverse Direction Applied): ");
    while (Serial.available() == 0);
    inputAngle = Serial.parseInt();
    clearSerialBuffer();
    if (inputAngle != -1) { 
        angle5 = inputAngle; 
        JointFive.write(map(angle5, 0, 270, 0, 180)); // Reverse direction & map range
    }
    // Serial.print("Joint 5 set to: "); Serial.println(angle5);

    // Serial.println("Enter angle for Joint 6 (0 to 270, or -1 to skip): ");
    // while (Serial.available() == 0);
    // inputAngle = Serial.parseInt();
    // clearSerialBuffer();
    // if (inputAngle != -1) { 
    //     angle6 = inputAngle;
    //     JointSix.write(map(angle6, 0, 270, 0, 180));
    // }
    // Serial.print("Joint 6 set to: "); Serial.println(angle6);

    delay(500); // Small delay before the next loop iteration
}
