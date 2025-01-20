/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// Function to set servo angle with calculated delay for SG90
int moveServoWithDelay(int currentAngle, int targetAngle) {
  // Calculate angle change
  int angleChange = abs(targetAngle - currentAngle);

  // Calculate delay time for SG90 servo (in milliseconds)
  int delayTime = angleChange * 3; // 0.12 sec for 60 degrees = 2 ms per degree

  // Move the servo to the target angle
  myservo.write(targetAngle);

  // Delay to allow the servo to reach the target position
  delay(delayTime);

  // Print the delay time for debugging
  Serial.print("Moving from ");
  Serial.print(currentAngle);
  Serial.print("° to ");
  Serial.print(targetAngle);
  Serial.print("°, delay time: ");
  Serial.print(delayTime);
  Serial.println(" ms");
  return targetAngle;
}


void setup() {
  myservo.attach(2);  // attaches the servo on pin 9 to the Servo object
  Serial.begin(9600);
}
int targetAngle = 0, currentAngle=0;
int dir = 0;
void testServo(){
  if(currentAngle > 180)
    dir = 1;
  if(currentAngle < 0)
    dir = 0;
  if(dir)
    targetAngle -= 10;
  else 
    targetAngle += 10;
  currentAngle = moveServoWithDelay(currentAngle, targetAngle);
}
void loop() {
  testServo();                 // sets the servo position according to the scaled value
  delay(1000);                           // waits for the servo to get there
}
