#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Set motor ports M2 (left) , M1 (right)
Adafruit_DCMotor *motor_right = AFMS.getMotor(1);
Adafruit_DCMotor *motor_left = AFMS.getMotor(2);

// Set LED pins and blinking time variables
int blueLED = 6;
bool blueLEDStatus = 0;
unsigned long currentLEDMillis;
unsigned long startLEDMillis;

// Global flags for block status
int blockNumber = 0;
int pickup = 0;
int blockType = 0;    // all green for testing

// Global flags for motor status
int RightMotorSpeed = 0;
int* RightMotorSpeedPter = &RightMotorSpeed;
int LeftMotorSpeed = 0;
int* LeftMotorSpeedPter = &LeftMotorSpeed;


// Set line sensors to input pins
int lineleftPin = 2;
int linerightPin = 3;
int lineSideRightPin = 4;
int lineSideLeftPin = 5;
int valLeft = digitalRead(lineleftPin); // read left input value
int valRight = digitalRead(linerightPin); // read right input value
int valSideRight = digitalRead(lineSideRightPin); // read side right input value
int valSideLeft = digitalRead(lineSideLeftPin); // read side left input value
// finger covering line sensor, line sensor lighting up means a reading of 1

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
          "RSR",              // 14: green to finish
          "LL"};              // 15: red to finish
      // no need junctionrotation('R') (u turn): 

  // REMEMBER TO CHANGE SIZEOFROUTES ARRAY AFTER CHANGING ROUTES

// number of junctions of each route, used when passing as second argument to routefollow()
const int sizeOfRoutes[16] = {2,3,4,3,4,3,3,5,5,2,3,3,3,4,3,2};
int routePtr;

// Set speed constants
const int HighSpeed = 250;            // adjustment on straight line
const int NormalSpeed = 200;          // straight line
const int LowSpeed = 50;               // adjustment on straight line
const int RotationSpeed = 180;        // rotation

// Junction to outpost (green red area) time in milliseconds
int junctionOutpostTime = 250 / NormalSpeed * 1000; // change arbitrary constant '400' // Higher normal speed means shorter time 

// Junction to finish box time in milliseconds
int junctionFinishTime = 550 / NormalSpeed * 1000;

// Leave junction time (make sure junction is not detected twice)
int leaveJunctionTime = 500;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Line following test");

  pinMode(lineleftPin, INPUT);
  pinMode(linerightPin, INPUT); 
  pinMode(lineSideRightPin, INPUT); 
  pinMode(lineSideLeftPin, INPUT); 
  pinMode(blueLED, OUTPUT);


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
    delay(30);

    // Serial.println("Arrived side branch");

}

void stopmoving() {
  updateleftmotorspeed(0);
  updaterightmotorspeed(0);
  digitalWrite(blueLED, LOW); // turn off blue LED
  blueLEDStatus = 0; 
  delay(30);
}

void gostraight() {   // walk in straight line 
  valLeft = digitalRead(lineleftPin); // read left input value
  valRight = digitalRead(linerightPin); // read right input value
  valSideRight = digitalRead(lineSideRightPin); // read side right input value
  valSideLeft = digitalRead(lineSideLeftPin); // read side left input value

  // update millis for blinking
  currentLEDMillis = millis();

  if(currentLEDMillis - startLEDMillis > 250) {
    blueLEDStatus = !blueLEDStatus;
    digitalWrite(blueLED, blueLEDStatus);
    startLEDMillis = millis();
  }


  if(valLeft == 1 && valRight == 1) { // both white
    // Serial.println("Go straight");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
    // Serial.println(*LeftMotorSpeedPter);

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
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
  }
  delay(10);        // we need delay for the motor to respond
  // digitalWrite(blueLED, LOW); // blinking blue led

}

void forwardawhile(int time) {
    unsigned long currentMotorMillis = millis();
    unsigned long startMotorMillis = millis();
    while((currentMotorMillis - startMotorMillis) < time) {
      currentMotorMillis = millis();
      gostraight();
    }
    stopmoving(); 
}

void routefollow(const char route[], int numberOfJunctions) {
  
  for (int currentJunction = 0; currentJunction < numberOfJunctions; currentJunction++) {
    
    while (valSideRight == 0 && valSideLeft == 0) {                     // go to junction
      gostraight();
    }
    delay(50);                                      // make sure the front sensors are sufficiently far away from junction
    stopmoving();

    // Serial.println("current junction");
    // Serial.println(route[currentJunction]);
    junctionrotation(route[currentJunction]);       // arrives junction, rotate

    // increment currentJunction counter
  }

  if (pickup) {
    forwardawhile(junctionOutpostTime);
  } 
  if ((blockNumber == 2 || blockNumber == 3) && !pickup) { // in open area
    forwardawhile(junctionOutpostTime);
    // need to change to function for approaching block in open area
  } 
  if ((blockNumber == 0 || blockNumber == 1) && !pickup) {
    // search for end of line
    while (valLeft == 1 || valRight == 1) {           // stops at the end of route where both front sensors detect black
      gostraight();                                   
    } 
    delay(80);                                       // make sure it goes further away from the white line
    stopmoving(); 
  }
  if ((blockNumber == 4) && !pickup) {
    // stop at finish box
    forwardawhile(junctionFinishTime);
  }


}

void liftblock() {
  pickup = 1;
  return;
}

void release() {
  pickup = 0;
  return;
}

void identifyblock() {
  blockNumber += 1;
  blockType = 0;
  return;
}

void loop() {
  // in C++, size of char = number of letters + 1
  routePtr = (blockNumber - 1) * 4 + !pickup * 2 + blockType;

  if (routePtr < 0) {                   // from start to block 1
    routefollow("LR", 2);
    liftblock();
    identifyblock();
  } else if (routePtr >= 14) {          // return to finish
    junctionrotation('R');
    int numberOfJunctions = sizeOfRoutes[routePtr];
    routefollow(routes[routePtr], numberOfJunctions);
    while (1) {                         // stop after finish
      stopmoving();
    }    
  } else {
    if (!pickup || blockNumber == 1 || blockNumber == 2) { // use flag for this line 
      junctionrotation('R');
    }
    int numberOfJunctions = sizeOfRoutes[routePtr];
    routefollow(routes[routePtr], numberOfJunctions);
    if (pickup) {
      release();
    } else {
      liftblock();
      identifyblock();
    }
  }


  
  Serial.print("blockNumber");
  Serial.println(blockNumber);
  Serial.print("blockType");
  Serial.println(blockType);
  Serial.print("pickup");
  Serial.println(pickup);
  Serial.print("routeptr");
  Serial.println(routePtr);

  
}
