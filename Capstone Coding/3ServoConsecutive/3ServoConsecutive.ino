#include <Servo.h>

// Create three Servo objects
Servo servo1;
Servo servo2;
Servo servo3;

// Assign each servo to a different control pin
int servoPin1 = 9;
int servoPin2 = 10;
int servoPin3 = 11;

void setup() {
  // Initialize serial communication for debugging/logging
  Serial.begin(9600);
  
  // Attach each servo object to its respective pin
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);

  // Note: Ensure each servo is powered properly. 
  // If your servos draw more current than the Arduino can supply, 
  // use an external power supply (still 5V, or as required) and connect all grounds together.
}

void loop() {
  // -------------------------------
  // TEST SERVO 1
  // -------------------------------
  Serial.println("Testing Servo 1...");

  // Full sweep from 0° up to 270° (adjust if your servo only supports 180°)
  for(int angle = 0; angle <= 270; angle+=30) {
    servo1.write(angle);
    
    delay(500);
  }

  // Full sweep from 270° down to 0°
  for(int angle = 270; angle >= 0; angle-=30) {
    servo1.write(angle);
    delay(500);
  }

  // // Map analog value (0–1023) from A0 to 0–180 servo angle
  // {
  //   int sensorValue = analogRead(A0);
  //   int mappedAngle = map(sensorValue, 0, 1023, 0, 180);
  //   servo1.write(mappedAngle);

  //   Serial.print("[Servo 1] Analog reading: ");
  //   Serial.print(sensorValue);
  //   Serial.print(" => Mapped angle: ");
  //   Serial.println(mappedAngle);
  // }

  // Pause before testing next servo
  delay(1000);

  // -------------------------------
  // TEST SERVO 2
  // -------------------------------
  Serial.println("Testing Servo 2...");

  // Full sweep from 0° up to 270° (adjust if your servo only supports 180°)
  for(int angle = 0; angle <= 270; angle+=30) {
    servo2.write(angle);
    
    delay(500);
  }

  // Full sweep from 270° down to 0°
  for(int angle = 270; angle >= 0; angle-=30) {
    servo2.write(angle);
    delay(500);
  }
  // // Map analog value (0–1023) from A0 to 0–180 servo angle
  // {
  //   int sensorValue = analogRead(A0);
  //   int mappedAngle = map(sensorValue, 0, 1023, 0, 180);
  //   servo2.write(mappedAngle);

  //   Serial.print("[Servo 2] Analog reading: ");
  //   Serial.print(sensorValue);
  //   Serial.print(" => Mapped angle: ");
  //   Serial.println(mappedAngle);
  // }

  // Pause before testing next servo
  delay(1000);
}
  // -------------------------------
  // TEST SERVO 3
  // -------------------------------
  // Serial.println("Testing Servo 3...");

  // // Full sweep from 0° up to 270°
  // for(int angle = 0; angle <= 270; angle++) {
  //   servo3.write(angle);
  //   delay(20);
  // }

//   // Full sweep from 270° down to 0°
//   for(int angle = 270; angle >= 0; angle--) {
//     servo3.write(angle);
//     delay(20);
//   }

//   // Map analog value (0–1023) from A0 to 0–180 servo angle
//   {
//     int sensorValue = analogRead(A0);
//     int mappedAngle = map(sensorValue, 0, 1023, 0, 180);
//     servo3.write(mappedAngle);

//     Serial.print("[Servo 3] Analog reading: ");
//     Serial.print(sensorValue);
//     Serial.print(" => Mapped angle: ");
//     Serial.println(mappedAngle);
//   }

//   // Final delay before repeating everything
//   delay(10000);
// }
