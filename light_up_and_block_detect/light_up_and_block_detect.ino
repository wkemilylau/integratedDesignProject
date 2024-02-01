#include <Servo.h>

Servo myservo; // create servo object to control a servo
bool pickup = 0; // boolean variable to track the state
int startAngle = 180; // initial start angle
int endAngle = 0; // initial end angle

// Define constants for maximum range and ADC solution accuracy
#define maxRange 520        // The max measurement value of the module is 520cm (a little bit longer than the effective max range)
#define adcSolution 1023.0  // ADC accuracy of Arduino UNO is 10-bit

// Define analog input pin for the ultrasonic sensor
const int sensityPin = A0;

// Define LED setup to be used upon block detection
const int ledRed = 2;    // Pin number, change if necessary
const int ledGreen = 3;  // Pin number, change if necessary

// Constant boolean for block type (0 or false: foam, 1 or true: solid)
const bool blockType = 0;

// Constant for threshold distances. Change this based on electrical/mechanical team's recommendation
const int thresholdDistance = 3;

void setup() {
  Serial.begin(9600);       // Initialize Serial communication
  pinMode(ledRed, OUTPUT);  // Set Pin 2 as output for RED led (SOLID block)
  pinMode(ledGreen, OUTPUT);// Set Pin 3 as output for GREEN led (FOAM block)
  myservo.attach(3); // attaches the servo on pin 9 to the servo object
}

// Declare variables for distance and sensor reading
float dist, sensity;

// Function to control LED lighting
void lightled(int ledColour) { 
  digitalWrite(ledColour, HIGH);  // Turn on LED
  delay(6000);
  digitalWrite(ledColour, LOW);   // Turn off LED
  delay(2000);
}

// Function to detect the type of block based on distance
void detectblock(float distance, bool isFoam) {
  if (distance >= 0 && distance <= thresholdDistance) {
    // SOLID block
    Serial.print(distance, 0);
    Serial.println("cm, SOLID block");
    lightled(ledRed);
    if (!pickup) {
      // Pickup the block
      pickupblock();
    }
  } else if (distance > thresholdDistance && distance <= thresholdDistance + 3) {
    // FOAM block
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightled(ledGreen);
    if (pickup) {
      // Release the block
      releaseblock();
    }
  } else {
    // This code can be used as failsafe in the future, in case we try to handle the case where dist > 6.
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightled(ledGreen);
    if (pickup) {
      // Release the block
      releaseblock();
    }
  }
}

void pickupblock() {
  pickup = true; // Set the boolean to true indicating that the block is picked up

  // Rotate the servo from startAngle to endAngle
  for (int pos = startAngle; pos <= endAngle; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
}

void releaseblock() {
  pickup = 0; // Set the boolean to 0 indicating that the block is released

  // Rotate the servo from endAngle to startAngle
  for (int pos = endAngle; pos >= startAngle; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
}

void loop() {
  // Read the value from the sensor
  sensity = analogRead(sensityPin);

  // Convert sensor reading to distance using a linear mapping
  dist = sensity * maxRange / adcSolution;

  // Determine the type of block based on the distance reading
  // Possible improvements: amplify signal to allow a larger range of 'safe' values for correct detection?
  detectblock(dist, blockType);

  pickupblock();

  // Delay for a short period before the next iteration
  delay(500);
  
}
