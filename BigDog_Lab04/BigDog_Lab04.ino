/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  BigDog_Lab02.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  Arduino pin mappings: https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet#pins
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182

  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin

  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/

  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

//state LEDs connections
#define redLED 7            //red LED for displaying states
#define grnLED 5            //green LED for displaying states
#define ylwLED 6            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5, 6, 7};    //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick
#define rtStepPin 50 //right stepper motor step pin
#define rtDirPin 51  // right stepper motor direction pin
#define ltStepPin 52 //left stepper motor step pin
#define ltDirPin 53  //left stepper motor direction pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define max_speed 1500 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 1000;   //delay for printing data

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

//define values movement
float Wheel_Dist = 9.5 / 12.0; //feet
float Pi = 3.14159;
int posX = 0;
int posY = 0;
int currentAngle = 0;
String curDir = "";
int speed = 500;
int CONST_SPD = 500;
bool origin = false;
bool pathPlanning = false;

//define the constant and variables
#define FRONT 0
#define BANK 1
#define LEFT 2
#define RIGHT 3
#define numOfSens 4

//define values for lidar and sonar
uint16_t wait = 100;
int16_t ft_lidar = 8;
int16_t bk_lidar = 9;
int16_t lt_lidar = 10;
int16_t rt_lidar = 11;
int16_t lidar_pins[4] = {ft_lidar, bk_lidar, lt_lidar, rt_lidar};

//define the constant and variables
#define VELOCITY_TEMP(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)  // The ultrasonic velocity (cm/us) compensated by temperature
#define RIGHT 0
#define LEFT 1

int16_t rt_trigechoPin = 3;
int16_t lt_trigechoPin = 4;
int16_t trig_EchoPin[2] = { 3, 4 };
int leftLight;
int rightLight;
//uint16_t wait = 250;

//define values for sensor distances
//In order: Forward, Back, Left, Right, ForwardRight, ForwardLeft
int Sensor_Distances[6] = {100000, 100000, 100000, 100000, 100000, 100000};
bool Object_in_Range = false;


// Wall Follow
int wall_dist = 25;
int wall_state = 1;

//Positional Data
float robo_theta = 0;
float robo_x = 0;
float robo_y = 0;
float return_theta = 0;
float return_x = 0;
float return_y = 0;
int robo_state = 0;
//0 = forward, 1 = one wheel turn, 2 = spin

//Docking Data
int dock_state = 0;
int light_threshold = 150;
//0 = initial wall follow, 1 = light follow, 2 = obstacle avoidance, 3 = dock, 4 = returning home

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

void allOFF() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(leds[i], LOW);
  }
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper() {
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}

/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*
   This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

/*
   Helper function to move robot
*/
void moveR(float distance, int speed) {
  //  Serial.print("Right:  ");
  stepperRight.setCurrentPosition(0);//sets right motor position to 0
  int steps = 0;
  //First converts the distance we want to travel into number of steps
  if (distance > 0) { // when moving forward
    steps = convertFeetToSteps(distance, 0);
  } else { // when moving backward
    steps = convertFeetToSteps(distance, 1);
  }
  if(robo_state == 0) {
    robo_x = robo_x + distance*cos(robo_theta*Pi/180)/2; 
    robo_y = robo_y + distance*sin(robo_theta*Pi/180)/2;
  } else if(robo_state == 1) {
    float cur_theta = (360*distance)/(Wheel_Dist*Pi)/2;
    robo_theta = robo_theta + cur_theta;
  } else {
    float cur_theta = (360*distance)/(2*Wheel_Dist*Pi);
    float cur_x = Wheel_Dist*sin(cur_theta*Pi/180)/2;
    float cur_y = (Wheel_Dist/2) - Wheel_Dist*cos(cur_theta*Pi/180)/2;
    float cur_dist = sqrt(cur_x*cur_x + cur_y*cur_y);
    robo_x = robo_x + cur_dist*cos((robo_theta + cur_theta/2)*Pi/180); 
    robo_y = robo_y + cur_dist*sin((robo_theta + cur_theta/2)*Pi/180);
    robo_theta = robo_theta + cur_theta;
  }
  if(robo_theta > 2*Pi) {
    robo_theta = robo_theta - 2*Pi;
  }
  if(robo_theta < -2*Pi) { 
    robo_theta = robo_theta + 2*Pi;
  }
  stepperRight.moveTo(steps);//move number of steps forward relative to current position
  stepperRight.setSpeed(speed);//set right motor speed
  stepperRight.runSpeedToPosition();//move right motor
}
void moveL(float distance, int speed) {
  //  Serial.print("Left:  ");
  stepperLeft.setCurrentPosition(0);//sets right motor position to 0
  int steps = 0;
  if (distance > 0) { // when moving forward
    steps = convertFeetToSteps(distance, 0);
  } else { // when moving backward
    steps = convertFeetToSteps(distance, 1);
  }
  if(robo_state == 0) {
    robo_x = robo_x + distance*cos(robo_theta*Pi/180)/2; 
    robo_y = robo_y + distance*sin(robo_theta*Pi/180)/2;
  } else if(robo_state == 1) {
    float cur_theta = (360*distance)/(Wheel_Dist*Pi)/2;
    robo_theta = robo_theta - cur_theta;
  } else {
    float cur_theta = (360*distance)/(2*Wheel_Dist*Pi);
    float cur_x = Wheel_Dist*sin(cur_theta*Pi/180)/2;
    float cur_y = (Wheel_Dist/2) - Wheel_Dist*cos(cur_theta*Pi/180)/2;
    float cur_dist = sqrt(cur_x*cur_x + cur_y*cur_y);
    robo_x = robo_x + cur_dist*cos((robo_theta - cur_theta/2)*Pi/180); 
    robo_y = robo_y + cur_dist*sin((robo_theta - cur_theta/2)*Pi/180);
    robo_theta = robo_theta - cur_theta;
  }
  if(robo_theta > 2*Pi) {
    robo_theta = robo_theta - 2*Pi;
  }
  if(robo_theta < -2*Pi) { 
    robo_theta = robo_theta + 2*Pi;
  }
  stepperLeft.moveTo(steps);//move number of steps forward relative to current position
  stepperLeft.setSpeed(speed);//set left motor speed
  stepperLeft.runSpeedToPosition();//move right motor
}
// takes in distance in feet and convert it into steps
int convertFeetToSteps(float measurement, int direction) {
  if (direction == 0) { // checks if the robot is going forward or reverse
    // Diameter of wheel: 3.34 in
    // Circumference of wheel: 10.49 in
    // 10.04 in == 800 steps forward
    // 30.48 cm = 1 ft
    // 25.50 cm distance = 800 steps
    float ratio_cm = ((measurement * 30.48) / 25.5) * 800; // conversion using cm
    float ratio_in = 800.0 * ((measurement * 10.49) / 10.04); // conversion using in
    int converted = (ratio_cm + ratio_in) / 2.0;
    //    Serial.println(converted);
    return converted;
  }
  else {
    // 10.82 in == 800 steps backwards
    // 27.50 cm distance = 800 steps reverse
    float ratio_cm = ((measurement * 30.48) / 25.5) * 800;
    float ratio_in = 800.0 * ((measurement * 10.49) / 10.04);
    int converted = (ratio_cm + ratio_in) / 2.0;
    //    Serial.println(converted);
    return converted;
  }

}

//int convert(float measurement, int dir){
// /* ENCODER FEEDBACK
//   * One Rotation in Steps = 800
//   * One Rotation = 40 (ALL match)
//   * Half Rotation = 21 L 20 R (FWD/REV match)
//   * Quarter Rotation = 10 (ALL match)
//   * Two Rotation = 80 (ALL match)
//   *
//   * One Foot: Ticks = 47 L 46 R Steps = 896
//   * One Foot: Ticks = 45 L 46 R REV Steps = -896
//   * Two Feet: Ticks = 89 (ALL match)   Steps = 1792
// */
//  float steps = 896;
//  float stpPerTicFwd = 896.0/47.0;
//  float stpPerTicRev = 896.0/45.0;
//  float ticPerFootFwd = 47.0;
//  float ticPerFootRev = 45.0;
//
//  if(dir == 0){
//    int converted = steps*measurement;
//    return converted;
//  } else{
//    int converted = steps*measurement;
//    return converted;
//  }
//}

/*
  The pivot function will use AccelStepper to move one motor forward and halt the other motor to turn the robot
  based on the angle given
*/
void pivot(int direction) {
  robo_state = 2;
  //  Serial.println("Pivot");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  //Calculate the distance needed to travel for given rotation
  float angle = abs(direction);//ensures value is positive
  float percent_rot = (angle / 360); // returns a fraction of the angle out of 360 degrees
  float distance = 2* Wheel_Dist * Pi * percent_rot; // calculates the distance the motor will travel where Wheel_Dist is the distance between two of the motors
  //Choose which direction we are turning
  if (direction < 0) {// when ccw
    moveL(distance, speed); //moves only right motor
  }
  else { // when cw
    moveR(distance, speed); //moves only left motor
  }
  steppers.runSpeedToPosition();//run until the robot reaches the target
}

/*
  The spin() function will use AccelSteppoer to rotate both motor clockwise or
  counter closewise based on the angle given
*/
void spin(int direction) {
  robo_state = 1;
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  //Calculate the distance needed to travel for given rotation
  float angle = abs(direction);//ensures angle is positive
  float percent_rot = (angle / 360); //gives fraction of angle to full circle
  float distance = Wheel_Dist * Pi * percent_rot; //calculates circumference
  //Choose which direction we are turning
//  currentAngle += direction;
//  Serial.print("Angle: ");
//  Serial.println(currentAngle);
  if (direction > 0) {
    moveR(distance, speed); //set right motor cw
    moveL(-distance, speed); //set left motor ccw
    curDir = "pos";
  }
  else {
    moveR(-distance, speed); //set right motor ccw
    moveL(distance, speed); //set left motor cw
    curDir = "neg";
  }
  //  Serial.println(speed);
  steppers.runSpeedToPosition();//run until the robot reaches the target
}

/*
  The turn() function will use AccelStepper to rotate both motor forward or backwardsa
  at different speeds based on the angle given
*/
void turn(float diameter, int degree) {
  stepperRight.setCurrentPosition(0);//sets right motor position to 0
  stepperLeft.setCurrentPosition(0);//sets left motor position to 0
  float c1 = (diameter + Wheel_Dist) * Pi; // circumference of outer wheel
  float c2 = (diameter - Wheel_Dist) * Pi; // circumference of inner wheel
  //Calculate the distance needed to travel for given rotation for wheels
  float angle = abs(degree);
  float percent_rot = (angle / 360);
  float distance_1 = c1 * percent_rot; // arc length for the outer wheel
  float distance_2 = c2 * percent_rot; // arc length for the inner wheel
  long positions[2]; // Array of desired stepper positions
  //Choose which direction we are turning
  if (degree < 0) {
    positions[0] = convertFeetToSteps(distance_1, 0); //right motor absolute position with slower speed
    positions[1] = convertFeetToSteps(distance_2, 0); //left motor absolute position with faster speed
  }
  else {
    positions[0] = convertFeetToSteps(distance_2, 0); //right motor absolute position with faster speed
    positions[1] = convertFeetToSteps(distance_1, 0); //left motor absolute position with slower speed
  }
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}
/*
  The forward() function will use AccelStepper to move the robot forward for a certain distance inputted
  The wheels will move in the same direction at the same speed
*/
void forward(float distance) {
  robo_state = 0;
  //Serial.println("Forward");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  int steps = distance * 800; // converting distance value to steps
  moveR(distance, speed); //set right motor
  moveL(distance, speed); //set left motor
  steppers.runSpeedToPosition();
  if(currentAngle == 90){
    posY = posY + distance;
  } else if(currentAngle == -90){
    posY = posY - distance;
  } else if(currentAngle == 0){
    posX = posX - distance;
  } else if (currentAngle == 180){
    posX = posX - distance;
  } else {
    posX = posX + cos(currentAngle)*distance;
    posY = posY + sin(currentAngle)*distance;
  }
  //Serial.print("Pos Y: ");
  //Serial.println(posY);
  //runToStop();//run until the robot reaches the target
}
/*
  The reverse() function will use AccelStepper to move the robot backwards for a certain distance inputted.
  The wheels will move in the same direction a  t the same speed*/
void reverse(float distance) {
  robo_state = 0;
  Serial.println("Reverse");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  moveR(-distance, speed); //set right motor
  moveL(-distance, speed); //set left  motor
  runToStop();//run until the robot reaches the target
}
/*
  The stop() function will use AccelStepper to stop the robot of all action
*/
void stop() {
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  stepperLeft.stop();//stops left motor
  stepperRight.stop();//stops right motor
}


/* ENCODER FEEDBACK
    One Rotation in Steps = 800
    One Rotation = 40 (ALL match)
    Half Rotation = 21 L 20 R (FWD/REV match)
    Quarter Rotation = 10 (ALL match)
    Two Rotation = 80 (ALL match)

    One Foot: Ticks = 45 L 46 R (FWD/REV match)  Steps = 896
    Two Feet: Ticks = 89 (ALL match)   Steps = 1792
*/

/*
   The goToAngle() functions takes in an angle in degrees as an input and rotates the robot using the spin() function
*/
void goToAngle(int angle) {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  if (angle != 0) {
    spin(angle);
  }
}
/*
   The goToGoal() function takes in a x and y coordinate and moves the robot to that point with the most optimal distance
*/
void goToGoal(int x, int y) {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  float angle = ((atan2((float)y, (float)x)) * 180 / Pi); //calculate angle of POI
  goToAngle(angle);//robot will turn at given angle
  currentAngle = angle;
  float dist = sqrt(x * x + y * y); //distance of point in feet
  forward(dist);
  posX = x;
  posY = y;
}

void returnPos() {
  long currPos[2] = {posX, posY};
  int currAng = currentAngle;
  float dist = sqrt(pow(posX, 2) + pow(posY, 2)); //distance of point in feet

  reverse(dist);
  goToAngle(-currAng);

}

void randomWander() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  int maxVal = 1000; //maximum value for random number
  //  randomSeed(analogRead(0)); //generate a new random number each time called
  int randNumber = random(maxVal); //generate a random number up to the maximum value
  int randLR = random(0, 1);
  Serial.println("Wandering");
  // max distance = 5 feet
  // max angle = 180 degrees
  //  Serial.println(randNumber);
  if ((randNumber % 2) == 1) {
    //    Serial.print("odd ");
    float distance = randNumber / 600;
    if (distance < 1) {
      distance = 0.5;
    }
    //    Serial.println(distance);
    forward(distance);

  } else if ((randNumber % 2) == 0) {
    //    Serial.print("even ");
    float angle;
    if (randNumber > 180) {
      angle = (randNumber - 820);
      //        Serial.println(angle);
      if (angle < 1 && angle > 0) {
        angle = 90;
      } else if ( angle < 0 && angle > -1) {
        angle = -90;
      }
    } else {
      if (randLR == 0) {
        angle = -randNumber;
      } else {
        angle = randNumber;
      }
    }
    //    Serial.println(angle);
    spin(angle);
  }
}

int readLidar(uint16_t side) {
  int16_t t = pulseIn(lidar_pins[side], HIGH);
  int d; //distance to  object
  if (t == 0) {
    // pulseIn() did not detect the start of a pulse within 1 second.
    //Serial.println("timeout");
    d = 2000; //no object detected
  }
  else if (t > 1850)  {
    //Serial.println("timeout");
    d = 2000; //no object detected
  }
  else  {
    // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
    d = (t - 1000) * 3 / 40;

    // Limit minimum distance to 0.
    if (d < 0) {
      d = 0;
    }
  }
  //   Serial.print(d);
  // Serial.print(" cm, ");
  return d;
}

//this function will read the left or right sensor based upon input value
uint16_t readSonar(uint16_t side) {
  uint16_t distance;
  uint32_t pulseWidthUs;
  int16_t dist, temp, dist_in;

  pinMode(trig_EchoPin[side], OUTPUT);
  digitalWrite(trig_EchoPin[side], LOW);
  digitalWrite(trig_EchoPin[side], HIGH);  //Set the trig pin High
  delayMicroseconds(10);               //Delay of 10 microseconds
  digitalWrite(trig_EchoPin[side], LOW);   //Set the trig pin Low
  pinMode(trig_EchoPin[side], INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(trig_EchoPin[side], HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * VELOCITY_TEMP(20) / 2.0;  //The distance can be calculated according to the flight time of ultrasonic wave,/
  //and the ultrasonic sound speed can be compensated according to the actual ambient temperature
  dist_in = 0.394 * distance;  //convert cm to inches
  return distance;
}
void readSensors() {
  //Reads the lidar sensors
  for (int i = 0; i < 4; i++) {
    Sensor_Distances[i] = readLidar(i);
  }
  //Reads the left and right sonar
  Sensor_Distances[4] = readSonar(RIGHT) - 2;
  Sensor_Distances[5] = readSonar(LEFT) - 2;

  //Cuts values past these distances
  int active_range = 35;
  for (int i = 0; i < 6; i++) {
    if (Sensor_Distances[i] == 2000) {
      Sensor_Distances[i] = 0;
    } else if (Sensor_Distances[i] > active_range) {
      Sensor_Distances[i] = 0;
    }
    Serial.print(i);
    Serial.print(": ");
    Serial.print(Sensor_Distances[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void Halt() {
  digitalWrite(redLED, HIGH);//turn off red LED
  digitalWrite(grnLED, LOW);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  Object_in_Range = false;
  readSensors();
  //Checks every sensor to see if there is an object in range
  for (int i = 0; i < 6; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(Sensor_Distances[i]);
    if (Sensor_Distances[i] == 0) {
      Sensor_Distances[i] = 2000;
    }
    Serial.print(" ");
    //delay(10);
    if (Sensor_Distances[i] <= 10) {
      Object_in_Range = true;
    }
  }
  //Checks to see if there are any objects in range and if there aren't keep moving
  if (Object_in_Range == false) {
    forward(0.7);
  }
  else {
    stop();
  }
}

void Follow() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED
  readSensors();
  //Checks the values of the sensors and removes the ones out of range
  int active_range = 25;
  //Calculate the angle the robot needs to move
  float Y_distance = Sensor_Distances[0] - Sensor_Distances[1] + Sensor_Distances[4] * cos(Pi / 4) + Sensor_Distances[5] * cos(Pi / 4);
  float X_distance =  Sensor_Distances[3] - Sensor_Distances[2] +  Sensor_Distances[4] * sin(Pi / 4) - Sensor_Distances[5] * sin(Pi / 4);
  //Check to see if robot is too close to object
  for (int i = 0; i < 6; i++) {
    if (Sensor_Distances[i] <= 8 && Sensor_Distances[i] != 0) {
      return;
    }
  }
  float dist = sqrt(pow(X_distance, 2) + pow(Y_distance, 2)); //distance of point in cm
  // if (Y_distance != 0)  {
  //   float angle = ((atan2((float)X_distance, (float)Y_distance)) * 180 / Pi); //calculate angle of POI
  //   goToAngle((int)angle);//robot will turn at given angle
  //   Serial.println(angle);
  // } else if (X_distance == 0 && Y_distance < 0) {
  //   float angle = -180; //calculate angle of POI
  //   goToAngle((int)angle);//robot will turn at given angle
  //   Serial.println(angle);
  // }
  float angle = ((atan2((float)X_distance, (float)Y_distance)) * 180 / Pi); //calculate angle of POI
  goToAngle((int) - angle); //robot will turn at given angle
  //  Serial.println();
  //  Serial.println(dist);
  //Uses distance from object to calc speed
  if (dist <= 8) {
    return;
  }
  speed = dist / active_range * 10000;
  if (speed == 0) {
    speed = CONST_SPD;
  }
  forward(dist / active_range * 0.4);
  speed = CONST_SPD;

  delay(100);
}

void Flee() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED
  readSensors();
  //Checks the values of the sensors and removes the ones out of range
  int active_range = 25;
  //  Serial.println();
  //Boundary Conditions
  if (Sensor_Distances[0] == 0 && Sensor_Distances[1] == 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] != 0) {
    //    Serial.println("Side Walls");
    reverse(0.5);
    return;
  } else if (Sensor_Distances[0] != 0 && Sensor_Distances[1] != 0 && Sensor_Distances[2] == 0 && Sensor_Distances[3] == 0) {
    //    Serial.println("Front Walls");
    spin(90);
    reverse(0.5);
    return;
  } else if (Sensor_Distances[0] == 0 && Sensor_Distances[1] != 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] != 0) {
    //    Serial.println("Forward Free");
    forward(0.5);
    return;
  } else if (Sensor_Distances[0] != 0 && Sensor_Distances[1] == 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] != 0) {
    //    Serial.println("Back Free");
    reverse(0.5);
    return;
  } else if (Sensor_Distances[0] != 0 && Sensor_Distances[1] != 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] == 0) {
    //    Serial.println("Right Free");
    spin(90);
    reverse(0.5);
    return;
  } else if (Sensor_Distances[0] != 0 && Sensor_Distances[1] != 0 && Sensor_Distances[2] == 0 && Sensor_Distances[3] != 0) {
    //    Serial.println("Left Free");
    spin(-90);
    reverse(0.5);
    return;
  } else if (Sensor_Distances[0] != 0 && Sensor_Distances[1] != 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] != 0) {
    //    Serial.println("Trapped");
    return;
  }
  //Calculate the angle the robot needs to move
  float Y_distance = Sensor_Distances[0] - Sensor_Distances[1] + Sensor_Distances[4] * cos(Pi / 4) + Sensor_Distances[5] * cos(Pi / 4);
  float X_distance =  Sensor_Distances[3] - Sensor_Distances[2] +  Sensor_Distances[4] * sin(Pi / 4) - Sensor_Distances[5] * sin(Pi / 4);
  float dist = sqrt(pow(X_distance, 2) + pow(Y_distance, 2)); //distance of point in feet
  //Check to see if there no object in range
  if (dist == 0) {
    return;
  }
  float angle = ((atan2((float)X_distance, (float)Y_distance)) * 180 / Pi); //calculate angle of POI
  goToAngle((int) - angle); //robot will turn at given angle
  //Uses distance from object to calc speed
  speed = dist / active_range * 1000;
  if (speed == 0) {
    speed = CONST_SPD;
  }
  //  Serial.print("Distance: ");
  //  Serial.println(active_range/dist*0.25);
  float distance = active_range / dist * 0.25;
  if (distance >= 1) {
    distance = 1;
  }
  reverse(distance);
  speed = CONST_SPD;

  delay(100);
}
void Smart_Follow() {
  readSensors();
  int active_range = 25;
  int danger_range = 8;
  for (int i = 0; i < 6; i++) {
    if (Sensor_Distances[i] == 2000) {
      Sensor_Distances[i] = 0;
    } else if (Sensor_Distances[i] > active_range) {
      Sensor_Distances[i] = 0;
    }
    if (Sensor_Distances[i] < active_range && Sensor_Distances[i] > danger_range) {
      Follow();
      return;
    } else if (Sensor_Distances[i] < danger_range && Sensor_Distances[i] > 0) {
      Halt();
      delay(1000);
      Flee();
      return;
    }
  }
  randomWander();
}
void Smart_Avoid() {
  readSensors();
  int active_range = 25;
  int danger_range = 8;
  for (int i = 0; i < 6; i++) {
    if (Sensor_Distances[i] == 2000) {
      Sensor_Distances[i] = 0;
    } else if (Sensor_Distances[i] > active_range) {
      Sensor_Distances[i] = 0;
    }
    if (Sensor_Distances[i] < active_range && Sensor_Distances[i] > danger_range) {
      Flee();
      return;
    } else if (Sensor_Distances[i] < danger_range && Sensor_Distances[i] > 0) {
      Halt();
      delay(1000);
      Flee();
      return;
    }
  }
  randomWander();
}

void wallFollow() {
  readSensors();
  //Runs if there are walls on each side
  if (Sensor_Distances[0] == 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] != 0) {
    //    Serial.println("Side Walls");
    if (Sensor_Distances[5] <= 10) {
      if (Sensor_Distances[5] <= 5) {
        spin(-20);
        forward(0.25);
      }
      float diff = Sensor_Distances[5] - Sensor_Distances[2];
      float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
      //      Serial.println(angle);
      spin(angle);
    }
    if (Sensor_Distances[4] <= 10) {
      if (Sensor_Distances[4] <= 5) {
        spin(20);
        forward(0.25);
      }
      float diff = Sensor_Distances[4] - Sensor_Distances[3];
      float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
      //      Serial.println(angle);
      spin(-angle);
    }
    forward(0.5);
    return;
  }
  //Runs if there is a wall to the left of the robot
  else if (Sensor_Distances[0] == 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] == 0) {
    Serial.println("Left Wall");
    digitalWrite(redLED, LOW);//turn off red LED
    digitalWrite(grnLED, HIGH);//turn on green LED
    digitalWrite(ylwLED, HIGH);//turn off yellow LED
    wall_state = 2;
    if (Sensor_Distances[5] <= 5) {
      spin(-20);
      forward(0.25);
    }
    if (Sensor_Distances[5] >= 30) {
      spin(20);
      forward(0.25);
    }
    float diff = Sensor_Distances[5] - Sensor_Distances[2];
    float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
    Serial.println(angle);
    spin(angle);
    forward(0.5);
    return;
  }
  //Runs if there is a wall to the right of the robot
  else if (Sensor_Distances[0] == 0 && Sensor_Distances[2] == 0 && Sensor_Distances[3] != 0) {
    wall_state = 3;
    Serial.println("Right Wall");
    digitalWrite(redLED, HIGH);//turn off red LED
    digitalWrite(grnLED, LOW);//turn on green LED
    digitalWrite(ylwLED, HIGH);//turn off yellow LED
    if (Sensor_Distances[4] <= 5) {
      spin(20);
      forward(0.25);
    }
    if (Sensor_Distances[4] >= 30) {
      spin(-20);
      forward(0.25);
    }
    float diff = Sensor_Distances[4] - Sensor_Distances[3];
    float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
    Serial.println(angle);
    spin(-angle);
    forward(0.5);
    return;
  }
  //Runs if there is a wall in front of the robot
  else if (Sensor_Distances[0] != 0) {
    digitalWrite(redLED, HIGH);//turn off red LED
    digitalWrite(grnLED, HIGH);//turn on green LED
    digitalWrite(ylwLED, LOW);//turn off yellow LED
    //Checks which wall we were on before the turn
    if (wall_state == 2) {
      Serial.println("Right turn");
      spin(-90);
      forward(0.5);
      return;
    } else {
      Serial.println("Left turn");
      spin(90);
      forward(0.5);
    }
  }
  //Check for blind turn
  else if (Sensor_Distances[0] == 0 && Sensor_Distances[2] == 0 && Sensor_Distances[3] == 0) {
    if (wall_state == 2) {
      Serial.println("Blind Left turn");
      forward(.75);
      spin(90);
      forward(.75);
      wall_state = 0;
      return;
    } else if (wall_state == 3) {
      Serial.println("Blind Right turn");
      forward(.75);
      spin(-90);
      forward(.75);
      wall_state = 0;
      return;
    } 
//    else if (pathPlanning){
//      forward(.5);
//    }
//    else {
//      randomWander();
//    }
  }
  if(posY == 0 && curDir == "neg" && pathPlanning == true){ //??? idk how to return to origin properly
    Serial.println("Return to origin");
    spin(-currentAngle);
    origin = true;
  }
}

void SmartGoal(int x, int y) {
  pathPlanning = true;
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  float angle = ((atan2((float)y, (float)x)) * 180 / Pi); //calculate angle of POI
  goToAngle(angle);//robot will turn at given angle
  float dist = sqrt(x * x + y * y); //distance of point in feet
  float curDist = 0.0;
// loops the forward function until it reaches the designated destination
  while (curDist < dist) {
    readSensors();
    origin = false;
    // checks if there is obstacle in front or sides, else it continues to move forward
    if (Sensor_Distances[0] != 0) {
      Serial.println("Object Detected");
      while(origin == false){
        wallFollow();
//        curDist += posX;
      }
    } 
//    else if ((Sensor_Distances[0] != 0 && Sensor_Distances[2] == 0 && Sensor_Distances[3] != 0) || (Sensor_Distances[0] != 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] == 0)) {
//      Serial.println("Multiple Object Detected");
//      while(origin == false){
//        wallFollow();
////        curDist += posX;
//      }
//    } 
//    else {
      forward(0.5); //probably need to use goToGoal but eh
      curDist = 0.5 + curDist;
      Serial.println(curDist);
//      origin = false;
//    }
  }
  delay(wait_time);
}
void Obstacle_Avoidance() {
readSensors();
  //Runs if there are walls on each side
  if (Sensor_Distances[0] == 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] != 0) {
    //    Serial.println("Side Walls");
    if (Sensor_Distances[5] <= 10) {
      if (Sensor_Distances[5] <= 5) {
        spin(-20);
        forward(0.25);
      }
      float diff = Sensor_Distances[5] - Sensor_Distances[2];
      float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
      //      Serial.println(angle);
      spin(angle);
    }
    if (Sensor_Distances[4] <= 10) {
      if (Sensor_Distances[4] <= 5) {
        spin(20);
        forward(0.25);
      }
      float diff = Sensor_Distances[4] - Sensor_Distances[3];
      float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
      //      Serial.println(angle);
      spin(-angle);
    }
    forward(0.5);
    return;
  }
  //Runs if there is a wall to the left of the robot
  else if (Sensor_Distances[0] == 0 && Sensor_Distances[2] != 0 && Sensor_Distances[3] == 0) {
    Serial.println("Left Wall");
    digitalWrite(redLED, LOW);//turn off red LED
    digitalWrite(grnLED, HIGH);//turn on green LED
    digitalWrite(ylwLED, HIGH);//turn off yellow LED
    wall_state = 2;
    if (Sensor_Distances[5] <= 5) {
      spin(-20);
      forward(0.25);
    }
    if (Sensor_Distances[5] >= 30) {
      spin(20);
      forward(0.25);
    }
    float diff = Sensor_Distances[5] - Sensor_Distances[2];
    float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
    Serial.println(angle);
    spin(angle);
    forward(0.5);
    return;
  }
  //Runs if there is a wall to the right of the robot
  else if (Sensor_Distances[0] == 0 && Sensor_Distances[2] == 0 && Sensor_Distances[3] != 0) {
    wall_state = 3;
    Serial.println("Right Wall");
    digitalWrite(redLED, HIGH);//turn off red LED
    digitalWrite(grnLED, LOW);//turn on green LED
    digitalWrite(ylwLED, HIGH);//turn off yellow LED
    if (Sensor_Distances[4] <= 5) {
      spin(20);
      forward(0.25);
    }
    if (Sensor_Distances[4] >= 30) {
      spin(-20);
      forward(0.25);
    }
    float diff = Sensor_Distances[4] - Sensor_Distances[3];
    float angle = ((atan2(diff, (float) 11)) * 180 / Pi); //calculate angle of POI
    Serial.println(angle);
    spin(-angle);
    forward(0.5);
    return;
  }
  //Runs if there is a wall in front of the robot
  else if (Sensor_Distances[0] != 0) {
    digitalWrite(redLED, HIGH);//turn off red LED
    digitalWrite(grnLED, HIGH);//turn on green LED
    digitalWrite(ylwLED, LOW);//turn off yellow LED
    //Checks which wall we were on before the turn
    if (wall_state == 2) {
      Serial.println("Right turn");
      spin(-90);
      forward(0.5);
      return;
    } else {
      Serial.println("Left turn");
      spin(90);
      forward(0.5);
    }
  }
  //Check for blind turn
  else if (Sensor_Distances[0] == 0 && Sensor_Distances[2] == 0 && Sensor_Distances[3] == 0) {
    if (wall_state == 2) {
      Serial.println("Blind Left turn");
      forward(0.5);
      dock_state = 1;
      wall_state = 0;
      return;
    } else if (wall_state == 3) {
      Serial.println("Blind Right turn");
      forward(0.5);
      dock_state = 1;
      return;
    } 
  }
}

void readLight() {
  leftLight = analogRead(1);
  rightLight = analogRead(2);
//  delay(1000);
}

void Love(){
  readLight();
  int minSpeed = 100;
  int maxSpeed = 750;
  int maxLightL = 550;
  int maxLightR = 650;
  int light_threshold = 150;
  float minDist = 0.5;
  
  int distLightL = (minDist*leftLight)/maxLightL;
  int speedL = (maxSpeed*maxLightL)/700;
  int distLightR = (minDist*rightLight)/maxLightR;
  int speedR = (maxSpeed*maxLightR)/700;
  //check if we are too close to either sensor
   if(leftLight > 450){
    dock_state = 3;
    return;
  }
  if (rightLight > 400){
    dock_state = 3;
    return;
  }

  //Set a cap on the speeds
  if(speedL > maxSpeed) {
    speedL = maxSpeed;
  } if(speedR > maxSpeed) {
    speedR = maxSpeed;
  }
  //Move forward at set speed if light threshold is met
  if(leftLight > light_threshold && rightLight <= light_threshold){
    robo_state = 2;
    moveR(0.1, speedL);
  }
  if (rightLight > light_threshold && leftLight <= light_threshold){
    robo_state = 2;
    moveL(0.1, speedR);
  }
  if(leftLight > light_threshold && rightLight > light_threshold) {
    robo_state = 1;
    moveL(0.1, speedR);
    moveR(0.1, speedL);
  }
  steppers.runSpeedToPosition();
}

void Fear(){
  readLight();
  int minSpeed = 500;
  int maxSpeed = 750;
  int maxLightL = 750;
  int maxLightR = 550;
  float minDist = 0.5;
  
  int distLightL = (minDist*leftLight)/maxLightL;
  int speedL = (maxSpeed*maxLightL)/700;
  int distLightR = (minDist*rightLight)/maxLightR;
  int speedR = (maxSpeed*maxLightR)/700;
  //check if we are too close to either sensor
   if(leftLight > maxLightL - 150){
    return;
  }
  if (rightLight > maxLightR){
    return;
  }

  //Set a cap on the speeds
  if(speedL > maxSpeed) {
    speedL = maxSpeed;
  } if(speedR > maxSpeed) {
    speedR = maxSpeed;
  }
  //Move forward at set speed if light threshold is met
  if(leftLight > light_threshold && rightLight <= light_threshold){
    robo_state = 2;
    moveL(0.1, speedL);
  }
  if (rightLight > light_threshold && leftLight <= light_threshold){
    robo_state = 2;
    moveR(0.1, speedR);
  } if(leftLight > light_threshold && rightLight > light_threshold) {
    robo_state = 1;
    moveL(0.1, speedL);
    moveR(0.1, speedR);
  }
  steppers.runSpeedToPosition();
}

void Explorer() {
  readLight();
  int minSpeed = 500;
  int maxSpeed = 750;
  int maxLightL = 750;
  int maxLightR = 550;
  float minDist = 0.5;
  
  int distLightL = (minDist*leftLight)/maxLightL;
  int speedL = (maxSpeed*maxLightL)/700;
  int distLightR = (minDist*rightLight)/maxLightR;
  int speedR = (maxSpeed*maxLightR)/700;
  //check if we are too close to either sensor
   if(leftLight > maxLightL - 150){
    return;
  }
  if (rightLight > maxLightR){
    return;
  }

  //Set a cap on the speeds
  if(speedL > maxSpeed) {
    speedL = maxSpeed;
  } if(speedR > maxSpeed) {
    speedR = maxSpeed;
  }
  //Move forward at set speed if light threshold is met
  moveL(0.1, speedL);
  moveR(0.1, speedR);
  steppers.runSpeedToPosition();
}

void Aggressor() {
  readLight();
  int minSpeed = 100;
  int maxSpeed = 750;
  int maxLightL = 550;
  int maxLightR = 650;
  int light_threshold = 150;
  int minDist = 1;
  robo_state = 2;
  
  int distLightL = (minDist*leftLight)/maxLightL;
  int speedL = (maxSpeed*maxLightL)/700;
  int distLightR = (minDist*rightLight)/maxLightR;
  int speedR = (maxSpeed*maxLightR)/700;
  if(leftLight > maxLightL - 100) {
    dock_state = 4;
    return;
  }
  if (rightLight > maxLightR - 100) {
    dock_state = 4;
    return;
  }

  //Set a cap on the speeds
  if(speedL > maxSpeed) {
    speedL = maxSpeed;
  } if(speedR > maxSpeed) {
    speedR = maxSpeed;
  }
  //Move forward at set speed if light threshold is met
  moveR(0.1*leftLight/maxLightL, speedR);
  moveL(0.1*rightLight/maxLightR, speedL);
  steppers.runSpeedToPosition();
}

void homing() {
  float target_x = robo_x - return_x;
  float target_y = -robo_y - return_y;
  double target_theta = atan2((double) target_x,(double) target_y);
  spin((-robo_theta - (float) target_theta) - 90);
  forward(0.5);
}

void hhoming() {
  readSensors();
  forward(0.5);
  if(Sensor_Distances[0]) {
    spin(-90);
    dock_state = 0;
  }
}

void Docking() {
  Serial.println(dock_state);
  readLight();
  readSensors();
  if(dock_state == 0) {
    if(leftLight >= light_threshold) {
      spin(55);
      forward(0.75);
      spin(15);
      dock_state = 1;
      return_x = robo_x;
      return_y = robo_y;
      return_theta = robo_theta;
    } 
    else if(rightLight >= light_threshold) {
      spin(-55);
      forward(0.75);
      spin(-15);
      dock_state = 1;
      return_x = robo_x;
      return_y = robo_y;
      return_theta = robo_theta;
    }
    else {
      Obstacle_Avoidance();
    }
  }
  else if(dock_state == 1) {
    if (Sensor_Distances[0] <= wall_dist || Sensor_Distances[2] <= wall_dist || Sensor_Distances[3] <= wall_dist) {
      if (Sensor_Distances[2] <= wall_dist && Sensor_Distances[2] != 0) {
        dock_state = 2;
      }
      else if (Sensor_Distances[3] <= wall_dist && Sensor_Distances[3] != 0) {
        dock_state = 2;
      }
      if(dock_state == 1) {
        Love();
      }
    }
  }
  else if(dock_state == 2) {
    Obstacle_Avoidance();
  }
  else if(dock_state == 3) {
    reverse(0.5);
    spin(180);
    delay(5000);
    dock_state = 4;
  }
  else if(dock_state == 4) {
    if (Sensor_Distances[2] <= wall_dist && Sensor_Distances[2] != 0) {
      Obstacle_Avoidance();
    }
    else if (Sensor_Distances[3] <= wall_dist && Sensor_Distances[3] != 0) {
      Obstacle_Avoidance();
    }
    else if (abs(robo_x - return_x) < 0.5 && abs(robo_y - return_y) < 0.5) {
      spin(90);
      dock_state = 0;
    }
    else {
      hhoming();
    }
  }
}

void TPF(String string) {
  int step = 0;
  for (int i = 0; i <= string.length(); i++) {
    while(step == i) {
      readSensors();
      if(string.charAt(i+1) == 'R') {
        if(Sensor_Distances[3] != 0) {
          step++;
          spin(-90);
        }
      }
      else if(string.charAt(i+1) == 'L') {
        if(Sensor_Distances[2] != 0) {
          step++;
          spin(90);
        }
      }
      else if(string.charAt(i+1) == 'T') {
        if(Sensor_Distances[0] != 0) {
          step++;
          return;
        }
      }
      if(step == i) {
        forward(0.5);
      }
    }
  }
}

void delimiter(String list){
  String newList[32];
  int count = 0;

  while(list.length() > 0){
    int i = list.indexOf(' ');
    // Checks if no space is found
    if(i == -1){
      newList[count++] = list;
      break;
    } else {
      newList[count++] = list.substring(0,i);
      list = list.substring(i+1);
    }
  }
  for(int i = 0; i < count; i++){
    Serial.print(i);
    Serial.print(": \"");
    Serial.print(newList[i]);
    Serial.println("\"");
  }
}

String x;
void GUI(){
  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    x = Serial.readString();
    // checks if string received is int
    if (isDigit(x.charAt(0))){
      // MAPPING
      if(x.length() == 16){
         digitalWrite(redLED, HIGH);
         digitalWrite(ylwLED, HIGH);//turn off yellow LED
         digitalWrite(grnLED, LOW);//turn off green LED
      }
      // PATH PLANNING
      if(x.length() == 7){
         digitalWrite(redLED, LOW);
         digitalWrite(ylwLED, HIGH);//turn off yellow LED
         digitalWrite(grnLED, HIGH);//turn off green LED        
      }
      // SENSORS
      if(x.length() == 6){
         digitalWrite(redLED, HIGH);
         digitalWrite(ylwLED, HIGH);//turn off yellow LED
         digitalWrite(grnLED, HIGH);//turn off green LED        
      }
    }
    if (x == "PL") {
      pivot(90);
    }
    if (x == "PR") {
      pivot(-90);
    }
    if (x == "F") {
      forward(0.5);
    }
    if (x == "B") {
      reverse(0.5);
    }
    if (x == "SL") {
      spin(90);
    }
    if (x == "SR") {
      spin(-90);
    }
    else{
      TPF(x); 
    }   
  }
}

//// MAIN
void setup() {
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor
  randomSeed(analogRead(0)); //generate a new random number each time called

  for (int i = 0; i < numOfSens; i++) {
    pinMode(lidar_pins[i], OUTPUT);
    delay(100);
  }

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  Serial.begin(baudrate);     //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
}

void loop() {
  Docking();
}
