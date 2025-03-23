#include <AccelStepper.h>

AccelStepper stepper(1, 9, 8);  // Step/Dir mode (Step Pin 9, Dir Pin 8)
#define ENA 7  // Enable Pin

// Function to calculate speed based on microstepping and desired RPM
void setMotorSpeed(int pulsesPerRev, float rpm) {
    float stepsPerSec = (pulsesPerRev * rpm) / 60.0;  // Convert RPM to steps/sec
    stepper.setMaxSpeed(stepsPerSec);  
    stepper.setAcceleration(stepsPerSec / 2);  // Smooth acceleration
}

void setup() {
    pinMode(ENA, OUTPUT);
    digitalWrite(ENA, HIGH);  // Enable driver

    int microstepping = 400;  // Set this to match your DM332T microstepping setting
    float desiredRPM = 300;  // Set your target speed in RPM

    setMotorSpeed(microstepping, desiredRPM);  // Auto-configure stepper speed
}

void loop() {
    stepper.moveTo(4800);  // Move X full rotations by dividing the # of steps by the microstepping resolution (if 400 pulses/rev then 4800 is 12 revolutions)
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }

    delay(250);

    stepper.moveTo(0);  // Move back to start
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }

    delay(250);
}
