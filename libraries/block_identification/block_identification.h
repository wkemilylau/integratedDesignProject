#ifndef BLOCK_IDENTIFICATION_H
#define BLOCK_IDENTIFICATION_H

#include <Arduino.h>
#include <Servo.h>

void lightled(int ledPin);
void detectblock(float distance);
void liftblock();
void releaseblock();


#endif