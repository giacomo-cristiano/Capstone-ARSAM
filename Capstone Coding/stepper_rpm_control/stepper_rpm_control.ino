#include <AccelStepper.h>

AccelStepper stepper(1, 7, 8);  // 1 = driver mode (Step + Dir)
#define ENA 7  // Enable pin

// --- Adjustable parameters --- //
const int microsteps = 400;         // From DM332T DIP switches
const int gearRatio = 100;          // Change to 20 if using 20:1 motor
const int pulsesPerRev = microsteps * gearRatio;

const float desiredRPM = 1.5;       // Conservative value for gearbox, adjust later
const int revolutionsToMove = 1;    // How many full output shaft revolutions

// -------------------------------- //

void setMotorSpeed(int ppr, float rpm) {
    float stepsPerSec = (ppr * rpm) / 60.0;
    stepper.setMaxSpeed(stepsPerSec);
    stepper.setAcceleration(stepsPerSec / 2);  // Smooth ramping
}

// JIG CONSTANTS 

#define RESET 40 
#define SLEEP 39

void setup() {
    pinMode(ENA, OUTPUT);
    digitalWrite(ENA, HIGH);  // Enable the driver (Assuming LOW is CCW)
// ARM STEPPER MOTORS 
    // pinMode(7, OUTPUT);  // STEP
    // pinMode(8 , OUTPUT);  // DIR

    // digitalWrite(8, LOW);  // Set direction LOW CCW BOTH GO UP Joint 2 and 3 
// Joint 2 (STEP is 7 and DIR is 8)
// Joint 3 (STEP is 9 and DIR is 10)
// Joint 1 (STEP is 11 and DIR is 12)
    // Pulse STEP pin a few times
    // for (int i = 0; i < 4000; i++) {
    //     digitalWrite(7, HIGH);
    //     delayMicroseconds(500);
    //     digitalWrite(7, LOW);
    //     delayMicroseconds(500);

    // JIG TESTING

    pinMode(RESET,OUTPUT);
    pinMode(SLEEP,OUTPUT);

    digitalWrite(RESET, HIGH);
    digitalWrite(SLEEP, HIGH);

    pinMode(29, OUTPUT);  // STEP
    pinMode(28, OUTPUT);  // DIR

    digitalWrite(28, LOW);  // Set direction LOW CCW BOTH GO UP Joint 2 and 3 
// (STEP is 29 and DIR is 28)

    // Pulse STEP pin a few times
    
    for (int i = 0; i < 4000; i++) {
        digitalWrite(29, HIGH);
        delayMicroseconds(500);
        digitalWrite(29, LOW);
        delayMicroseconds(500);
    }
}
    // setMotorSpeed(pulsesPerRev, desiredRPM);  // Set speed & accel

void loop() {
//     stepper.moveTo(pulsesPerRev * revolutionsToMove);  // Forward
//     while (stepper.distanceToGo() != 0) {
//         stepper.run();
//     }

//     delay(500);

//     stepper.moveTo(0);  // Return
//     while (stepper.distanceToGo() != 0) {
//         stepper.run();
//     }

//     delay(500);
}
