#include "block_identification.h"



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
