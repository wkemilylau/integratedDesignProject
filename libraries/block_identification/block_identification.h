#ifndef BLOCK_IDENTIFICATION_H
#define BLOCK_IDENTIFICATION_H

#include <Arduino.h>
#include <Servo.h>


Servo pincerServo; // create servo object to control a servo
bool pickup = 0; // boolean to track pickup state
int servoStartAngle = 0; // initial start angle
int servoEndAngle = 90; // initial end angle

// Define constants for maximum range and ADC solution accuracy
#define maxRange 520        // The max measurement value of the module is 520cm (a little bit longer than the effective max range)
#define adcSolution 1023.0  // ADC accuracy of Arduino UNO is 10-bit

// Define analog input pin for the ultrasonic sensor
const int usSensorPin = A0;

// Define LED setup to be used upon block detection, boolean flag for distance that determines solid/foam
const int redLED = 10;             // Pin number, change if necessary
const int greenLED = 11;           // Pin number, change if necessary
const int solidUpperBound = 3;
const int foamUpperBound = 6;


void lightled(int ledPin);
void detectblock(float distance);
void liftblock();
void releaseblock();

#endif
