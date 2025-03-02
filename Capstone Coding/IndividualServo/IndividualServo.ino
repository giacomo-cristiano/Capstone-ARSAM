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

  //mapping an analog value to servo angle (from a potentiometer on A0)
  int sensorValue = analogRead(A0); //reads the voltage value on pin A0
  int mappedAngle = map(sensorValue, 0, 1023, 0, 180); //maps that voltage to an angle
  myServo.write(mappedAngle);

  Serial.print("Analog reading: ");
  Serial.print(sensorValue);
  Serial.print(" => Mapped angle: ");
  Serial.println(mappedAngle);

  delay(10000); // 10 second before repeating the process
}
