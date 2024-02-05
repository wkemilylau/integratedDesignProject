#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Adafruit_MotorShield.h>
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_VL53L0X.h>

void updaterightmotorspeed(int NewRightMotorSpeed);
void updatelinesensors();
void junctionrotation(char Direction[2]);
void stopmoving();
void rotateright90degrees();
void gostraight();
void forwardawhile(int time);
void routefollow(const char route[], int numberOfJunctions);
void liftblock();
void release();
void identifyblock(); 


#endif