/*
Created by Emily Lau, Isaac Lam, February 2024
IDP
*/

#include "navigation/navigation.cpp"
#include "navigation/navigation.h"
#include "globalflags/globalflags.cpp"
#include "globalflags/globalflags.h"
#include "block_identification/block_identification.cpp"
#include "block_identification/block_identification.h"
#include "Arduino.h"

const int buttonPin = 9;        // BUTTON
int buttonVal = 0;              // BUTTON

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Line following test");
  
  // TOF DISTANCE SENSOR
  // join i2c bus (address optional for master)
  Wire.begin();
  // Set I2C sub-device address
  sensor.begin(0x50);
  // Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  // Laser rangefinder begins to work
  sensor.start();

  pinMode(lineleftPin, INPUT);
  pinMode(linerightPin, INPUT); 
  pinMode(lineSideRightPin, INPUT); 
  pinMode(lineSideLeftPin, INPUT); 
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);    // Set Pin 2 as output for RED led (SOLID block)
  pinMode(greenLED, OUTPUT);  // Set Pin 3 as output for GREEN led (FOAM block)
  pincerServo.attach(pincerServoPin);
  pincerServo.write(servoStartAngle);     

  pinMode(buttonPin, INPUT);  // BUTTON


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

  // turn on motor
  motor_left->run(RELEASE);
  motor_right->run(RELEASE); 
  
}

void loop() {
  // in C++, size of char = number of letters + 1

  if (start == 0) {                         // wait until button pushed

    buttonVal = digitalRead(buttonPin);
    
    if (buttonVal == HIGH) {
      start = 1;
    }

    

  // } else {

  //   routePtr = (blockNumber - 1) * 4 + !pickup * 2 + blockType;

  //   if (routePtr < 0) {                   // from start to block 1

  //     updatelinesensors();
  //     while (valLeft == 0 || valRight == 0) {   // leave start block
  //       gostraight();
  //       delay(5);
  //       updatelinesensors();
  //     }

  //     routefollow("SLR", 3);
  //     float usVal = analogRead(usSensorPin);
  //     float usDistance = usVal * maxRange  / adcSolution;
  //     Serial.println(usDistance);
  //     identifyblock();

  //   } else if (routePtr >= 14) {          // return to finish
  //     junctionrotation('R');
  //     int numberOfJunctions = sizeOfRoutes[routePtr];
  //     routefollow(routes[routePtr], numberOfJunctions);
  //     while (1) {                         // stop after finish
  //       stopmoving();
  //     }    
  //   } else {
  //     if (!pickup || blockNumber == 1 || blockNumber == 2) {           // u turn to leave outpost and residential area
  //       junctionrotation('R');                                         
  //     }
  //     int numberOfJunctions = sizeOfRoutes[routePtr];
  //     routefollow(routes[routePtr], numberOfJunctions);

  //     if ((blockNumber == 2 || blockNumber == 3) && !pickup) {         // arrived open area
  //       findandapproachblock(); 
  //       identifyblock();
  //       float usVal = analogRead(usSensorPin);
  //       float usDistance = usVal * maxRange  / adcSolution;
  //       Serial.println(usDistance);
  //       delay(500);
  //       returntoline();
  //     } else if (pickup) {                                             // arrived outpost
  //       releaseblock();                                                // ********COMMENT THIS IF WE ARE GIVING UP SOLID BLOCK*********
  //       // if (blockType == 0) {                                         // ********UNCOMMENT THIS IF WE ARE GIVING UP SOLID BLOCK*********
  //       //   releaseblock();
  //       // }
  //       backwardawhile(350);
  //     } else if ((blockNumber == 0 || blockNumber == 1) && !pickup) {  // arrived residential area                                                         
  //       identifyblock();
  //       float usVal = analogRead(usSensorPin);
  //       float usDistance = usVal * maxRange  / adcSolution;
  //       Serial.println(usDistance);
  //     }
  //   }

  // }



  // } else {
  //   updatelinesensors();
  //   while (valLeft == 0 || valRight == 0) {   // leave start block
  //     gostraight();
  //     delay(5);
  //     updatelinesensors();
  //   }



  //   routefollow("SLR", 3);
  //   float usVal = analogRead(usSensorPin);
  //   float usDistance = usVal * maxRange  / adcSolution;
  //   Serial.println(usDistance);
  //   identifyblock();
  //   while(1) {
  //   stopmoving();}
  // }

  } else {
    liftblock();
    delay(1000);
    releaseblock();
    start = 0;
  }


}
