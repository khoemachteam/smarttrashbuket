#include <Servo.h>

#define SERVO_PIN 2
#define TRIG_PIN 3  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN 4 // Pin connected to the Echo pin of the sensor

#define DISTANCE_OPEN 20 //cm
#define OPEN_ANGLE 180 //degree
#define CLOSE_ANGLE 0 //degree
#define DELAY_OPEN_TIME 500//ms

Servo myservo;  // create Servo object to control a servo
int currentAngle = 0; // Variable to track the current angle

int moveServoWithDelay(int currentAngle, int targetAngle);
long readUltrasonicDistance(int trigPin, int echoPin);
void playfulLidMovement(int cycles, int openAngle, int closeAngle, int speedDelay);
void playfulLidMovement2();
void playfulLidMovement3();
void setup() {
  Serial.begin(9600);
  myservo.attach(2);  // attaches the servo on pin 9 to the Servo object
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  currentAngle = moveServoWithDelay(currentAngle, CLOSE_ANGLE);
  playfulLidMovement2();
}

// int targetAngle = 0;
// int dir = 0;
// void testServo(){
//   if(currentAngle > 180)
//     dir = 1;
//   if(currentAngle < 0)
//     dir = 0;
//   if(dir)
//     targetAngle -= 10;
//   else 
//     targetAngle += 10;
//   currentAngle = moveServoWithDelay(currentAngle, targetAngle);
// }

void loop() {
  long dis = readUltrasonicDistance(TRIG_PIN, ECHO_PIN);
  Serial.println(dis);
  if(dis > 0){
    if(dis < DISTANCE_OPEN){
      currentAngle = moveServoWithDelay(currentAngle, OPEN_ANGLE);
      Serial.println("Wait for sometime to close lid");
      delay(DELAY_OPEN_TIME); // this sensor is not good to read continuously
      currentAngle = moveServoWithDelay(currentAngle, CLOSE_ANGLE);
      Serial.println("Closed lid");
      // playfulLidMovement3();
    }
  }
  delay(300); // this sensor is not good to read continuously
}

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

long readUltrasonicDistance(int trigPin, int echoPin) {
  // Set the timeout based on the maximum range (400 cm)
  const long timeout = 23300; // 23.3 ms for 400 cm round trip
  
  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Send a 10µs HIGH pulse
  digitalWrite(trigPin, LOW);

  // Read the echo pulse duration
  long duration = pulseIn(echoPin, HIGH, timeout);

  // Calculate the distance in cm
  long distance = (duration / 2) * 0.0343; // Divide by 2 for one-way trip

  // Return the distance (if no echo detected, return -1)
  if (duration == 0) {
    return -1; // Timeout occurred (no object detected within range)
  }
  return distance;
}


// Function to make the lid open and close repeatedly like a playful dog
void playfulLidMovement(int cycles, int openAngle, int closeAngle, int speedDelay) {
  for (int i = 0; i < cycles; i++) {
    currentAngle = moveServoWithDelay(currentAngle, openAngle); // Open the lid
    delay(speedDelay); // Quick pause while open
    currentAngle = moveServoWithDelay(currentAngle, closeAngle); // Close the lid
    delay(speedDelay); // Quick pause while closed

    // Debugging: Print current cycle info
    Serial.print("Playful movement cycle ");
    Serial.println(i + 1);
  }
}

void playfulLidMovement2(){
  playfulLidMovement(2, OPEN_ANGLE/2, CLOSE_ANGLE, 0);
  playfulLidMovement(1, OPEN_ANGLE/1, CLOSE_ANGLE, 0);
  playfulLidMovement(2, OPEN_ANGLE/2, CLOSE_ANGLE, 0);
}

void playfulLidMovement3(){
  playfulLidMovement(3, OPEN_ANGLE/2, CLOSE_ANGLE, 0);
}

void playfulLidMovement4(){
  playfulLidMovement(3, OPEN_ANGLE/1, CLOSE_ANGLE, 0);
}
