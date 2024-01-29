/*
   URM09 Block detection using Ultrasonic Sensor test
*/

// Define constants for maximum range and ADC solution accuracy
#define MAX_RANGE 520        // The max measurement value of the module is 520cm (a little bit longer than the effective max range)
#define ADC_SOLUTION 1023.0  // ADC accuracy of Arduino UNO is 10-bit

// Define analog input pin for the ultrasonic sensor
const int sensityPin = A0;

// Define LED setup to be used upon block detection
const int ledRed = 2;  // Pin number, change if necessary
const int ledGreen = 3;   // Pin number, change if necessary

void setup() {
  Serial.begin(9600);           // Initialize Serial communication
  pinMode(ledRed, OUTPUT);    // Set Pin 2 as output for RED led
  pinMode(ledGreen, OUTPUT);     // Set Pin 3 as output for GREEN led
}

// Declare variables for distance and sensor reading
float dist, sensity;

// Function to control LED lighting
void lightLED(int ledColour) {
  digitalWrite(ledColour, HIGH);  // Turn on LED
  delay(6000);
  digitalWrite(ledColour, LOW);   // Turn off LED
  delay(2000);
}

void loop() {
  // Read the value from the sensor
  sensity = analogRead(sensityPin);

  // Convert sensor reading to distance using a linear mapping
  dist = sensity * MAX_RANGE / ADC_SOLUTION;

  // Determine the type of block based on the distance reading
  // Possible improvements: amplify signal to allow a larger range of 'safe' values for correct detection?

  // Code below has been tested for distance and with LEDs
  if (dist >= 0 && dist <= 3) {
    // BLACK block
    Serial.print(dist, 0);
    Serial.println("cm, BLACK block");
    lightLED(ledRed);
  } else if (dist > 3 && dist <= 6) {
    // BLUE block
    Serial.print(dist, 0);
    Serial.println("cm, BLUE block");
    lightLED(ledGreen);
  } else {
    // 50-50 chance, assume blue block lol
    Serial.print(dist, 0);
    Serial.println("cm, BLUE block");
    lightLED(ledGreen);    
  }

  // Delay for a short period before the next iteration
  delay(500);
}
