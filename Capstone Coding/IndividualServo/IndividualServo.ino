#include <Servo.h>

Servo myServo;           // Create servo object
int servoPin = 9;        // Servo signal pin, apparently this is the digital pin to control the servo's signal line,
                         // pin 9 supports pulse width modulation signals

void setup() {
  Serial.begin(9600); //initializes the communication in bps, print messages to serial monitor, sensor readings
  myServo.attach(servoPin); //links to pin 9 on the arduino
}

void loop() {
  //Full sweep from 0° up to 180°
  for(int angle = 0; angle <= 270; angle++) {
    myServo.write(angle); //write angle to the servo motor
    delay(20);  // Small delay in ms between angle updates for a smooth sweep 
  }

  //Full sweep from 180° down to 0°
  for(int angle = 270; angle >= 0; angle--) {
    myServo.write(angle);
    delay(20);
  }

}
