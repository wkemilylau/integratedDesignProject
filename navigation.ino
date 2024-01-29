#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Set motor ports M2 (left) , M1 (right)
Adafruit_DCMotor *motor_right = AFMS.getMotor(1);
Adafruit_DCMotor *motor_left = AFMS.getMotor(2);

// Global flags for motor status
int RightMotorSpeed = 0;
int* RightMotorSpeedPter = &RightMotorSpeed;
int LeftMotorSpeed = 0;
int* LeftMotorSpeedPter = &LeftMotorSpeed;

// Set line sensors to input pins
int lineleftPin = 2;
int linerightPin = 3;
int lineSideRightPin = 4;
int valLeft = digitalRead(lineleftPin); // read left input value
int valRight = digitalRead(linerightPin); // read right input value
int valSideRight = digitalRead(lineSideRightPin); // read side right input value
// finger covering line sensor, line sensor lighting up means a reading of 1

// array of routes; does not include backing out in free space; does not include U-turns after picking block up in line area
char routes[16][6] = {           
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


// Set speed constants
const int HighSpeed = 150;            // adjustment on straight line
const int NormalSpeed = 100;          // straight line
const int LowSpeed = 0;               // adjustment on straight line
const int RotationSpeed = 100;        // rotation

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Line following test");

  pinMode(lineleftPin, INPUT);
  pinMode(linerightPin, INPUT); 
  pinMode(lineSideRightPin, INPUT); 


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
    Serial.println("start to rotate");
    
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
      while (valSideRight == 1) {
        gostraight();
      }
      return;
    } else {
      Serial.println("BAD INPUT");
      while (1) {
        stopmoving();
      }
    }
    
    // rotate until both of the front sensors detect the side branch
    while (valLeft == 1 || valRight == 1) {    // rotate until front sensors are both black
      Serial.println("Rotating until front sensors are both black");

      valLeft = digitalRead(lineleftPin); // read left input value
      valRight = digitalRead(linerightPin); // read right input value
      delay(5);
    }

    while (valLeft == 0 || valRight == 0) {   // rotate until front sensors are both white
      Serial.println("Rotating until front sensors are both white");
      valLeft = digitalRead(lineleftPin); // read left input value
      valRight = digitalRead(linerightPin); // read right input value
      delay(5);
    }

    valSideRight = digitalRead(lineSideRightPin); // read side right input value
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(0);
    updaterightmotorspeed(0);
    delay(1000);

    Serial.println("Arrived side branch");

}

void stopmoving() {
  updateleftmotorspeed(0);
  updaterightmotorspeed(0);
  delay(1000);
}

void gostraight() {   // walk in straight line 
  valLeft = digitalRead(lineleftPin); // read left input value
  valRight = digitalRead(linerightPin); // read right input value
  valSideRight = digitalRead(lineSideRightPin); // read side right input value

  if(valLeft == 1 && valRight == 1) { // both white
    // Serial.println("Go straight");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
    Serial.println(*LeftMotorSpeedPter);

  } else if(valLeft == 0 && valRight == 1) { // left is black right is white
    // Serial.println("Turn right");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(HighSpeed);
    updaterightmotorspeed(LowSpeed);
    Serial.println(*LeftMotorSpeedPter);

  } else if(valLeft == 1 && valRight == 0) { // left is white right is black
    // Serial.println("Turn left");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(LowSpeed);
    updaterightmotorspeed(HighSpeed);
    Serial.println(*LeftMotorSpeedPter);

  } else { // both black
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
  }
  delay(10);        // we need delay for the motor to respond
}

void routefollow(const char route[], int numberOfJunctions) {
  
  for (int currentJunction = 0; currentJunction < numberOfJunctions; currentJunction++) {
    
    while (valSideRight == 0) {                     // go to junction
      gostraight();
    }
    stopmoving();

    Serial.println("current junction");
    Serial.println(route[currentJunction]);
    junctionrotation(route[currentJunction]);       // arrives junction, rotate

    // increment currentJunction counter
  }

  // if not in open area indicated by flag:
  while (valLeft == 1 || valRight == 1) {           // stops at the end of route where both front sensors detect black
    gostraight();                                   
  } 
  //

  stopmoving(); 
    

}

void loop() {
  // in C++, size of char = number of letters + 1



  char testroute[3] = "SH";                                                     // start to block 1   

  int numberOfJunctions = sizeof(testroute) / sizeof(testroute[0]) - 1;
  routefollow(testroute, numberOfJunctions);


  delay(100000);
}

