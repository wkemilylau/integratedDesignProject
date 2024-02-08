/*
Library for navigation
Created by Emily Lau, Isaac Lam, February 2024
IDP
*/
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include "../globalflags/globalflags.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Set motor ports M2 (left) , M1 (right)
Adafruit_DCMotor *motor_right = AFMS.getMotor(1);
Adafruit_DCMotor *motor_left = AFMS.getMotor(2);

// Set LED pins and blinking time variables
int blueLED = 12;
bool blueLEDStatus = 0;
unsigned long currentLEDMillis;
unsigned long startLEDMillis;

// // Global flags for block status
// int blockNumber = 0;
// bool pickup = 0;
// bool blockType = 1;    
// bool inOpenArea = 0;

// Global flags for motor status
int RightMotorSpeed = 0;
int* RightMotorSpeedPter = &RightMotorSpeed;
int LeftMotorSpeed = 0;
int* LeftMotorSpeedPter = &LeftMotorSpeed;

// Delay time in ms for rotateright90degrees
const int rotate90Offset = 180;

// Time between line and back wall in open area
const int lineToBackWall = 2000;

// Distances in open area
const int sideWallDistance = 300;
const int backWallDistance = 510;                   // value between back wall and front face of block
const int leaveWallDistance = 2000;
const int sensorWheelDistance = 700;

// Set line sensors to input pins
int lineleftPin = 2;
int linerightPin = 3;
int lineSideRightPin = 13;
int lineSideLeftPin = 8;
int valLeft = digitalRead(lineleftPin); // read left input value
int valRight = digitalRead(linerightPin); // read right input value
int valSideRight = digitalRead(lineSideRightPin); // read side right input value
int valSideLeft = digitalRead(lineSideLeftPin); // read side left input value
// finger covering line sensor, line sensor lighting up means a reading of 1

// // Set ultrasonic and ToF sensors to input pins
DFRobot_VL53L0X sensor; 
// int usDistance = sensor.getDistance();
// int tofDistance = sensor.getDistance();



// array of routes; does not include backing out in free space; does not include U-turns after picking block up in line area
const char routes[16][6] = {  

  // REMEMBER TO CHANGE SIZEOFROUTES ARRAY AFTER CHANGING ROUTES

          "RL",               // 0: block 1 to green 
          "LSR",              // 1: block 1 to red
          "SRSR",             // 2: green to block 2
          "SLL",              // 3: red to block 2
          "LSLS",             // 4: block 2 to green
          "RRS",              // 5: block 2 to red
          "SSS",              // 6: green to block 3
          "SLSRR",            // 7: red to block 3
          "RSSLS",            // 8: block 3 to green
          "SS",               // 9: block 3 to red
          "SSR",              // 10: green to block 4
          "SSL",              // 11: red to block 4
          "RLS",              // 12: block 4 to green
          "LSRS",             // 13: block 4 to red
          "RSRS",              // 14: green to finish
          "LLS"};              // 15: red to finish
      // no need junctionrotation('R') (u turn): 

  // REMEMBER TO CHANGE SIZEOFROUTES ARRAY AFTER CHANGING ROUTES

// number of junctions of each route, used when passing as second argument to routefollow()
const int sizeOfRoutes[16] = {2,3,4,3,4,3,3,5,5,2,3,3,3,4,4,3};
int routePtr;

// Set speed constants
const int HighSpeed = 250;            // adjustment on straight line
const int NormalSpeed = 200;          // straight line
const int LowSpeed = 50;               // adjustment on straight line
const int RotationSpeed = 180;        // rotation
const int OpenAreaSpeed = 100;        // slow speed in open area

// Junction to outpost (green red area) time in milliseconds
int junctionOutpostTime = 250 / NormalSpeed * 1000; // change arbitrary constant '400' // Higher normal speed means shorter time 

// Junction to finish box time in milliseconds
int junctionFinishTime = 200 / NormalSpeed * 1000;

// Leave junction time (make sure junction is not detected twice)
int leaveJunctionTime = 300;

// functions
void updaterightmotorspeed(int NewRightMotorSpeed);
void updateleftmotorspeed(int NewLeftMotorSpeed);
void updatelinesensors();
void junctionrotation(char Direction[2]);
void stopmoving();
void rotateright90degrees();
void gostraight();
void forwardawhile(int time);
void findandapproachblock();
void returntoline();
void routefollow(const char route[], int numberOfJunctions);
// void liftblock();
// void release();
// void identifyblock();




#endif