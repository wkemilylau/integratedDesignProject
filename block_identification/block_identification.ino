#include <Servo.h>

Servo pincerServo; // create servo object to control a servo
bool pickup = 0; // boolean to track pickup state
int servoStartAngle = 0; // initial start angle
int servoEndAngle = 90; // initial end angle

// Define constants for maximum range and ADC solution accuracy
#define maxRange 520        // The max measurement value of the module is 520cm (a little bit longer than the effective max range)
#define adcSolution 1023.0  // ADC accuracy of Arduino UNO is 10-bit

// Define analog input pin for the ultrasonic sensor
const int ultrasonicSensorPin = A0;

// Define LED setup to be used upon block detection, boolean flag for distance that determines solid/foam 
const int redLED = 2;             // Pin number, change if necessary
const int greenLED = 3;           // Pin number, change if necessary
const int thresholdDistance = 3;  // Change this based on electrical/mechanical team's recommendation

void setup() {
  Serial.begin(9600);         // Initialize Serial communication
  pinMode(redLED, OUTPUT);    // Set Pin 2 as output for RED led (SOLID block)
  pinMode(greenLED, OUTPUT);  // Set Pin 3 as output for GREEN led (FOAM block)
  pincerServo.attach(9);          // attaches the servo on pin 9 to the servo object
}

// Declare variables for distance and ultrasonic sensor reading
float ultrasonic_dist, ultrasonic_sensor_reading;

// Function to control LED lighting
void lightled(int ledPin) { 
  digitalWrite(ledPin, HIGH);  // Turn on LED
  delay(6000);
  digitalWrite(ledPin, LOW);   // Turn off LED
  delay(2000);
}

// Function to detect the type of block based on distance
void detectblock(float distance) {
  if (distance >= 0 && distance <= thresholdDistance) {
    // SOLID block
    Serial.print(distance, 0);
    Serial.println("cm, SOLID block");
    lightled(redLED);
    if (!pickup) {
      // Pickup the block
      liftblock();
    }
  } else if (distance > thresholdDistance && distance <= thresholdDistance + 3) {
    // FOAM block
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightled(greenLED);
    if (pickup) {
      // Release the block
      releaseblock();
    }
  } else {
    // This code can be used as failsafe in the future, in case we try to handle the case where dist > 6.
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightled(greenLED);
    if (pickup) {
      // Release the block
      releaseblock();
    }
  }
}

void liftblock() {
  pickup = 1; // Set the boolean to true indicating that the block is picked up

  // Rotate the servo from servoStartAngle to servoEndAngle
  for (int pos = servoStartAngle; pos <= servoEndAngle; pos += 1) {
    pincerServo.write(pos);
    delay(15);
  }
}

void releaseblock() {
  pickup = 0; // Set the boolean to 0 indicating that the block is released

  // Rotate the servo from servoEndAngle to servoStartAngle
  for (int pos = servoEndAngle; pos >= servoStartAngle; pos -= 1) {
    pincerServo.write(pos);
    delay(15);
  }
}

void loop() {
  // Read the value from the sensor
  ultrasonic_sensor_reading = analogRead(ultrasonicSensorPin);

  // Convert sensor reading to distance using a linear mapping
  ultrasonic_dist = ultrasonic_sensor_reading * maxRange / adcSolution;

  // Determine the type of block based on the distance reading
  // Possible improvements: amplify signal to allow a larger range of 'safe' values for correct detection?
//  detectblock(ultrasonic_dist);

  liftblock();
  delay(500);
  releaseblock();

  // Delay for a short period before the next iteration
  delay(500);
  
}
