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

int count = 0;    // for testing, count number of junctions
int* countPtr = &count;

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
                                              // Direction == "R" means turn right
    updateleftmotorspeed(RotationSpeed);      // set constant rotate speed
    updaterightmotorspeed(RotationSpeed);
    
    if (Direction == "R") {
      motor_left->run(FORWARD);
      motor_right->run(BACKWARD);
    } else if (Direction == "L") {
      motor_left->run(BACKWARD);
      motor_right->run(FORWARD);
    } else {
      motor_left->run(FORWARD);
      motor_right->run(FORWARD);
      return;
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

    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(0);
    updaterightmotorspeed(0);
    delay(1000);

    Serial.println("Arrived side branch");
    *countPtr += 1;
}

void loop() {
  valLeft = digitalRead(lineleftPin); // read left input value
  valRight = digitalRead(linerightPin); // read right input value
  valSideRight = digitalRead(lineSideRightPin); // read side right input value

  if(valLeft == 1 && valRight == 1) { // both white
    Serial.println("Go straight");

    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
    Serial.println(*LeftMotorSpeedPter);

  } else if(valLeft == 0 && valRight == 1) { // left is black right is white
    Serial.println("Turn right");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(HighSpeed);
    updaterightmotorspeed(LowSpeed);
    Serial.println(*LeftMotorSpeedPter);

  } else if(valLeft == 1 && valRight == 0) { // left is white right is black
    Serial.println("Turn left");
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(LowSpeed);
    updaterightmotorspeed(HighSpeed);
    Serial.println(*LeftMotorSpeedPter);

  } else { // both black
      if (*countPtr == 2) {
        updateleftmotorspeed(0);
        updaterightmotorspeed(0);
      } else {
        Serial.println("almost arrive junction");            // should be close to a junction, slow down
        motor_left->run(FORWARD);
        motor_right->run(FORWARD);
        updateleftmotorspeed(NormalSpeed);
        updaterightmotorspeed(NormalSpeed);
        Serial.println(*LeftMotorSpeedPter);
        Serial.println(0);
      }
  }

  if(valSideRight == 1){ // white branch on the right
    Serial.println("Branch on the right, turn right");
    
    if (count == 0) {
      junctionrotation("L");
    } else if (count == 1) {
      junctionrotation("R");
    }
    
  }

  delay(10);
}

