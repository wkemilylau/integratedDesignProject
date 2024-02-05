#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

DFRobot_VL53L0X sensor; 
int distanceInMillimeters; // Variable to store distance in millimeters

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
  
  //Need to decide how to move ahead with detection logic

  delay(500);
}
