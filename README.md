

Here’s a README file for your project:

---

# Servo Control with Ultrasonic Sensor

This project demonstrates how to control a servo motor using an ultrasonic distance sensor (HC-SR04). The servo moves based on the distance measured by the sensor, and the lid opens when an object is detected within a certain range and closes after a predefined delay.

## Components Required:
- SG90 Servo Motor
- HC-SR04 Ultrasonic Distance Sensor
- Arduino Board (e.g., Arduino Uno)

## Pin Connections:
- Servo Motor:
  - Signal pin -> Pin 2 (SERVO_PIN)
- Ultrasonic Sensor:
  - TRIG pin -> Pin 3 (TRIG_PIN)
  - ECHO pin -> Pin 4 (ECHO_PIN)

## Features:
- Opens the servo at 180° when an object is detected within a set distance.
- Closes the servo after a delay (`DELAY_OPEN_TIME`) once the object is no longer in range.
- The servo’s movement is delayed to ensure it operates smoothly.

## Code Explanation:

### Global Variables:
- **SERVO_PIN**: Pin connected to the servo motor.
- **TRIG_PIN**: Pin connected to the TRIG pin of the ultrasonic sensor.
- **ECHO_PIN**: Pin connected to the ECHO pin of the ultrasonic sensor.
- **DISTANCE_OPEN**: Distance threshold (in cm) for triggering the servo to open.
- **OPEN_ANGLE**: Angle (in degrees) for opening the servo.
- **CLOSE_ANGLE**: Angle (in degrees) for closing the servo.
- **DELAY_OPEN_TIME**: Time delay (in ms) to keep the servo open.

### Functions:
1. **moveServoWithDelay()**: Moves the servo from the current angle to the target angle with a calculated delay based on the SG90 servo's speed.
2. **readUltrasonicDistance()**: Reads the distance from the ultrasonic sensor using the TRIG and ECHO pins.

### Setup:
In the `setup()` function, the servo and ultrasonic sensor pins are initialized, and the servo is initially moved to the closed position.

### Loop:
In the `loop()` function:
- The distance is continuously measured.
- If the distance is less than `DISTANCE_OPEN`, the servo moves to the open position.
- After a delay of `DELAY_OPEN_TIME`, the servo closes.
- If the distance is greater than the threshold, the servo remains closed.

### Servo Speed:
The speed of the SG90 servo is considered by calculating the delay time between angles (approximately 2 ms per degree).

## Usage:
- Upload the code to your Arduino.
- The servo will open when the ultrasonic sensor detects an object within the set distance, and the servo will close after a brief delay.

## Notes:
- Ensure your servo is connected to the appropriate pin (2 in this case).
- Adjust the **DISTANCE_OPEN**, **OPEN_ANGLE**, **CLOSE_ANGLE**, and **DELAY_OPEN_TIME** constants to suit your needs.

---

This README provides the necessary information to understand and use your servo control project.
