/* 
Library for global flags
Created by Emily Lau, Isaac Lam, February 2024
IDP
*/

#ifndef GLOBALFLAGS_H
#define GLOBALFLAGS_H
#include "Arduino.h"

// Global flags for block status
int blockNumber = 0;        // updates after identifying and picking up block
bool pickup = 0; 
bool blockType = 0;         // 0 for foam, 1 for solid
bool inOpenArea = 0;
bool start = 0;

#endif