#include <Servo.h>

Servo myservo; // create servo object to control a servo
bool pickup = 0; // boolean variable to track the state
int START_ANGLE = 180; // initial start angle
int END_ANGLE = 0; // initial end angle

// Define constants for maximum range and ADC solution accuracy
#define MAX_RANGE 520        // The max measurement value of the module is 520cm (a little bit longer than the effective max range)
#define ADC_SOLUTION 1023.0  // ADC accuracy of Arduino UNO is 10-bit

// Define analog input pin for the ultrasonic sensor
const int sensityPin = A0;

// Define LED setup to be used upon block detection
const int LED_RED = 2;    // Pin number, change if necessary
const int LED_GREEN = 3;  // Pin number, change if necessary

// Constant boolean for block type (0 or false: foam, 1 or true: solid)
const bool blockType = 0;

// Constant for threshold distances. Change this based on electrical/mechanical team's recommendation
const int THRESHOLD_DIST = 3;

void setup() {
  Serial.begin(9600);       // Initialize Serial communication
  pinMode(LED_RED, OUTPUT);  // Set Pin 2 as output for RED led (SOLID block)
  pinMode(LED_GREEN, OUTPUT);// Set Pin 3 as output for GREEN led (FOAM block)
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
  if (distance >= 0 && distance <= THRESHOLD_DIST) {
    // SOLID block
    Serial.print(distance, 0);
    Serial.println("cm, SOLID block");
    lightled(LED_RED);
    if (!pickup) {
      // Pickup the block
      pickupblock();
    }
  } else if (distance > THRESHOLD_DIST && distance <= THRESHOLD_DIST + 3) {
    // FOAM block
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightled(LED_GREEN);
    if (pickup) {
      // Release the block
      releaseBlock();
    }
  } else {
    // This code can be used as failsafe in the future, in case we try to handle the case where dist > 6.
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightled(LED_GREEN);
    if (pickup) {
      // Release the block
      releaseBlock();
    }
  }
}

void pickupblock() {
  pickup = true; // Set the boolean to true indicating that the block is picked up

  // Rotate the servo from START_ANGLE to END_ANGLE
  for (int pos = START_ANGLE; pos <= END_ANGLE; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
}

void releaseBlock() {
  pickup = 0; // Set the boolean to 0 indicating that the block is released

  // Rotate the servo from END_ANGLE to START_ANGLE
  for (int pos = END_ANGLE; pos >= START_ANGLE; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
}

void loop() {
  // Read the value from the sensor
  sensity = analogRead(sensityPin);

  // Convert sensor reading to distance using a linear mapping
  dist = sensity * MAX_RANGE / ADC_SOLUTION;

  // Determine the type of block based on the distance reading
  // Possible improvements: amplify signal to allow a larger range of 'safe' values for correct detection?
  detectblock(dist, blockType);

  pickupblock();

  // Delay for a short period before the next iteration
  delay(500);
  
}
