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

// Set speed constants
const int HighSpeed = 150;
const int NormalSpeed = 100;
const int LowSpeed = 0;

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
    Serial.println("almost arrive junction");            // should be close to a junction, slow down
    motor_left->run(FORWARD);
    motor_right->run(FORWARD);
    updateleftmotorspeed(NormalSpeed);
    updaterightmotorspeed(NormalSpeed);
    Serial.println(*LeftMotorSpeedPter);
    Serial.println(0);

  }

  if(valSideRight == 1){ // white branch on the right
    Serial.println("Branch on the right, turn right");
    
    // rotate until both of the front sensors detect the side branch
    // before rotation, front sensors can detect white or black depending on the shape of the junction
    while (valLeft == 1 || valRight == 1) {    // rotate until front sensors are both black
      Serial.println("Rotating until front sensors are both black");
      motor_left->run(FORWARD);
      motor_right->run(BACKWARD);
      updateleftmotorspeed(100);      // set constant rotate speed
      updaterightmotorspeed(100);
      valLeft = digitalRead(lineleftPin); // read left input value
      valRight = digitalRead(linerightPin); // read right input value
      delay(5);
    }

    while (valLeft == 0 || valRight == 0) {   // rotate until front sensors are both white
      Serial.println("Rotating until front sensors are both white");
      motor_left->run(FORWARD);
      motor_right->run(BACKWARD);
      updateleftmotorspeed(100);
      updaterightmotorspeed(100);
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

    
  }

  delay(10);
}

