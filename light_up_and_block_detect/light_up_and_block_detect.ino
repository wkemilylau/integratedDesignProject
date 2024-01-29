/*
   URM09 Block detection using Ultrasonic Sensor test
*/

// Define constants for maximum range and ADC solution accuracy
#define MAX_RANGE 520        // The max measurement value of the module is 520cm (a little bit longer than the effective max range)
#define ADC_SOLUTION 1023.0  // ADC accuracy of Arduino UNO is 10-bit

// Define analog input pin for the ultrasonic sensor
const int sensityPin = A0;

// Define LED setup to be used upon block detection
const int ledRed = 2;    // Pin number, change if necessary
const int ledGreen = 3;  // Pin number, change if necessary

// Constant boolean for block type (false: foam, true: solid)
const bool IS_SOLID_DEFAULT = false;

void setup() {
  Serial.begin(9600);       // Initialize Serial communication
  pinMode(ledRed, OUTPUT);  // Set Pin 2 as output for RED led (SOLID block)
  pinMode(ledGreen, OUTPUT);// Set Pin 3 as output for GREEN led (FOAM block)
}

// Declare variables for distance and sensor reading
float dist, sensity;

// Function to control LED lighting
void lightLED(int ledColour) { /
  digitalWrite(ledColour, HIGH);  // Turn on LED
  delay(6000);
  digitalWrite(ledColour, LOW);   // Turn off LED
  delay(2000);
}

// Function to detect the type of block based on distance
void detectBlock(float distance, bool isFoam) {
  if (distance >= 0 && distance <= 3) {
    // SOLID block
    Serial.print(distance, 0);
    Serial.println("cm, SOLID block");
    lightLED(ledRed);
  } else if (distance > 3 && distance <= 6) {
    // FOAM block
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightLED(ledGreen);
  } else {
    // This code can be used as failsafe in the future, in case we try to handle the case where dist > 6.
    Serial.print(distance, 0);
    Serial.println("cm, FOAM block");
    lightLED(ledGreen);
  }
}

void loop() {
  // Read the value from the sensor
  sensity = analogRead(sensityPin);

  // Convert sensor reading to distance using a linear mapping
  dist = sensity * MAX_RANGE / ADC_SOLUTION;

  // Determine the type of block based on the distance reading
  // Possible improvements: amplify signal to allow a larger range of 'safe' values for correct detection?
  detectBlock(dist, IS_SOLID_DEFAULT);

  // Delay for a short period before the next iteration
  delay(500);
}
