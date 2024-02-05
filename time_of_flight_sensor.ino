#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Set motor ports M2 (left) , M1 (right)
Adafruit_DCMotor *motor_right = AFMS.getMotor(1);
Adafruit_DCMotor *motor_left = AFMS.getMotor(2);
DFRobot_VL53L0X sensor; 

int distanceInMillimeters; // Variable to store distance in millimeters
// Global flags for motor status
int RightMotorSpeed = 0;
int* RightMotorSpeedPter = &RightMotorSpeed;
int LeftMotorSpeed = 0;
int* LeftMotorSpeedPter = &LeftMotorSpeed;

void setup() {
    // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // join i2c bus (address optional for master)
  Wire.begin();
  // Set I2C sub-device address
  sensor.begin(0x50);
  // Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  // Laser rangefinder begins to work
  sensor.start();
    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start
  motor_right->setSpeed(*RightMotorSpeedPter);
  motor_left->setSpeed(*LeftMotorSpeedPter);
  motor_right->run(FORWARD);
  motor_left->run(FORWARD);

  //turn on motor
  motor_left->run(RELEASE);
  motor_right->run(RELEASE); 

}

void loop() {
  // Get the distance in millimeters
  distanceInMillimeters = sensor.getDistance();
  // Convert millimeters to centimeters
  float distanceInCentimeters = distanceInMillimeters / 10.0;
  // Print the distance in centimeters
  Serial.print("Distance: ");
  Serial.print(distanceInCentimeters);
  Serial.println(" cm");

  if (distanceInCentimeters < 50) {
    motor_right->setSpeed(100);
    motor_left->setSpeed(100);
    motor_right->run(FORWARD);
    motor_left->run(FORWARD);
  }
  else {
    motor_right->setSpeed(100);
    motor_left->setSpeed(100);
    motor_right->run(BACKWARD);
    motor_left->run(BACKWARD);
  }
  //Need to decide how to move ahead with detection logic

  delay(500);
}
