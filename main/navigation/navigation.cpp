#include "navigation.h"


void updaterightmotorspeed(int NewRightMotorSpeed) {    // update right motor speed when it is different from the current speed
  if (NewRightMotorSpeed == *RightMotorSpeedPter) {
    return;
  }
  else {
    *RightMotorSpeedPter = NewRightMotorSpeed;
    motor_right->setSpeed(NewRightMotorSpeed);
  }
}

void updateleftmotorspeed(int NewLeftMotorSpeed) {    // update left motor speed when it is different from the current speed
  if (NewLeftMotorSpeed == *LeftMotorSpeedPter) {
    return;
  }
  else {
    *LeftMotorSpeedPter = NewLeftMotorSpeed;
    motor_left->setSpeed(NewLeftMotorSpeed);
  }
}

void updatelinesensors() {
  valLeft = digitalRead(lineleftPin); // read left input value
  valRight = digitalRead(linerightPin); // read right input value
  valSideRight = digitalRead(lineSideRightPin); // read side right input value
  valSideLeft = digitalRead(lineSideLeftPin); // read side left input value
}

void junctionrotation(char Direction[2]) {    // C++ requires 2 spaces to store 1 character
                                              // Direction == "R" means turn right, "L" means left, "S" (or other char) means straight line
    // Serial.println("start to rotate");
    
    updateleftmotorspeed(RotationSpeed);      // set constant rotate speed
    updaterightmotorspeed(RotationSpeed);
    
    if (Direction == 'R') {
      motor_left->run(FORWARD);
      motor_right->run(BACKWARD);
    } else if (Direction == 'L') {
      motor_left->run(BACKWARD);
      motor_right->run(FORWARD);
    } else if (Direction == 'S') {
      motor_left->run(FORWARD);
      motor_right->run(FORWARD);
      forwardawhile(leaveJunctionTime);         // make sure robot leaves junction by some distance
      return;
    } else if (Direction == '\0'){    // reaches the end (null terminator) of the input character // NOT SURE IF ITS CORRECT
      stopmoving();       
      return;
    } else {
      // Serial.println("BAD INPUT");
      while (1) {
        stopmoving();
      }
    }
    
    // rotate until both of the front sensors detect the side branch
    while (valLeft == 1 || valRight == 1) {    // rotate until front sensors are both black
      // Serial.println("Rotating until front sensors are both black");

      valLeft = digitalRead(lineleftPin); // read left input value
      valRight = digitalRead(linerightPin); // read right input value
      delay(5);
    }

    while (valLeft == 0 || valRight == 0) {   // rotate until front sensors are both white
      // Serial.println("Rotating until front sensors are both white");
      valLeft = digitalRead(lineleftPin); // read left input value
      valRight = digitalRead(linerightPin); // read right input value
      delay(5);
    }

    forwardawhile(leaveJunctionTime);                     // make sure robot leaves junction by some distance

    valSideRight = digitalRead(lineSideRightPin); // read side right input value
    valSideLeft = digitalRead(lineSideLeftPin); // read side right input value
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    stopmoving();

    // Serial.println("Arrived side branch");

}

void stopmoving() {
  updateleftmotorspeed(0);
  updaterightmotorspeed(0);
  digitalWrite(blueLED, LOW); // turn off blue LED
  blueLEDStatus = 0; 
  delay(30);
}

void rotateright90degrees() {
    Serial.println("Rotate right");
    updateleftmotorspeed(RotationSpeed);      // set constant rotate speed
    updaterightmotorspeed(RotationSpeed);
    motor_left->run(FORWARD);
    motor_right->run(BACKWARD);

    updatelinesensors();

    while (valSideLeft == 0 && valSideRight == 0) {   // rotate until side sensors are both white
      // Serial.println("Rotating until front sensors are both white");
      valSideLeft = digitalRead(lineSideLeftPin); // read left input value
      valSideRight = digitalRead(lineSideRightPin); // read right input value
      delay(5);
      Serial.println("not yet reach target");
    }
    delay(rotate90Offset);
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    delay(30);
}

void gostraight() {   // walk in straight line 
  updatelinesensors();
  currentLEDMillis = millis();      // update millis for blinking

  if(currentLEDMillis - startLEDMillis > 250) {             // 250 ms --> 2 Hz blinking rate
    blueLEDStatus = !blueLEDStatus;
    digitalWrite(blueLED, blueLEDStatus);
    startLEDMillis = millis();
  }

  if(valLeft == 1 && valRight == 1) { // both white
    // Serial.println("Go straight");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);  
    if (inOpenArea == 1) {                     // in open area, detecting block while following white line
    updateleftmotorspeed(OpenAreaSpeed);
    updaterightmotorspeed(OpenAreaSpeed);
    } else {
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
    }

  } else if(valLeft == 0 && valRight == 1) { // left is black right is white
    // Serial.println("Turn right");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(HighSpeed);
    updaterightmotorspeed(LowSpeed);
    // Serial.println(*LeftMotorSpeedPter);

  } else if(valLeft == 1 && valRight == 0) { // left is white right is black
    // Serial.println("Turn left");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(LowSpeed);
    updaterightmotorspeed(HighSpeed);
    // Serial.println(*LeftMotorSpeedPter);

  } else { // both black
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    if (inOpenArea == 1) {                           // in open area, approaching block, in black area no line
    updateleftmotorspeed(OpenAreaSpeed);
    updaterightmotorspeed(OpenAreaSpeed);
    } else {
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
    }
  }
  delay(10);        // we need delay for the motor to respond
}

void forwardawhile(int time) {
    Serial.println("forward a while");
    unsigned long currentMotorMillis = millis();
    unsigned long startMotorMillis = millis();
    while((currentMotorMillis - startMotorMillis) < time) {
      currentMotorMillis = millis();
      gostraight();
    }
    stopmoving(); 
}

void findandapproachblock() {
  inOpenArea = 1;         // tells gostraight() that we are in open area so need to move slowly
  int tofDistance = sensor.getDistance();

  while (tofDistance > sideWallDistance) {         //x: distance of wall, to be tested
    gostraight();
    tofDistance = sensor.getDistance();
  }
  stopmoving();   // indicates wall detected
  delay(1000);
  forwardawhile(leaveWallDistance);                // leave wall by a sufficient distance (moves slowly)
  tofDistance = sensor.getDistance();
  while (tofDistance > backWallDistance) {         //y: distance of back wall of building minus block
    gostraight();
    tofDistance = sensor.getDistance();
  }
  forwardawhile(sensorWheelDistance);    // (to be tested) distance between sensor and center of rotation
  rotateright90degrees();
  inOpenArea = 0;
  forwardawhile(lineToBackWall);   // goes forward until reach back wall  
}

void returntoline() {
  // go backward until one of the side sensors touch white, turn left
  updatelinesensors();

  motor_left->run(BACKWARD);
  motor_right->run(BACKWARD);
  updateleftmotorspeed(NormalSpeed);
  updaterightmotorspeed(NormalSpeed);
  while (valSideRight == 0 && valSideLeft == 0) {         // keep moving backward until either side sensor touches white
    updatelinesensors();
  }
  stopmoving();

  junctionrotation('L');              // turn left to go back on white line
}

void routefollow(const char route[], int numberOfJunctions) {
  
  for (int currentJunction = 0; currentJunction < numberOfJunctions; currentJunction++) {
    
    while (valSideRight == 0 && valSideLeft == 0) {                     // go to junction
      gostraight();
    }
    delay(50);                                      // make sure the front sensors are sufficiently far away from junction
    stopmoving();
    junctionrotation(route[currentJunction]);       // arrives junction, rotate
  }

  if (pickup) {
    forwardawhile(junctionOutpostTime);
  } else if ((blockNumber == 0 || blockNumber == 1) && !pickup) {
    // search for end of line
    while (valLeft == 1 || valRight == 1) {           // stops at the end of route where both front sensors detect black
      gostraight();                                   
    } 
    delay(80);                                       // make sure it goes further away from the white line
    stopmoving(); 
  } else if ((blockNumber == 4) && !pickup) {
    // stop at finish box
    forwardawhile(junctionFinishTime);
  }

  return;


}

// void liftblock() {
//   pickup = 1;
//   Serial.print("lift block");
//   return;
// }

// void release() {
//   pickup = 0;
//   Serial.println("release block");
//   return;
// }

// void identifyblock() {
//   blockNumber += 1;
//   blockType = 0;
//   return;
// }
