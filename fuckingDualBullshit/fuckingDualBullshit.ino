//Justin Dewitt 12.12.23
// read about multicore: https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core
//This code will show how to run the same code on the M4 and M7 to server and client, respectively
//M4 will read sensor data and send to M7 to run the state machine

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include "Arduino.h"
#include "RPC.h"

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
#define LEFT_EN 0        //left encoder
#define RIGHT_EN 1       //right encoder
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
int speed = 1000;

//define the constant and variables
#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define numOfSens 4
#define RIGHT_SON 0
#define LEFT_SON 1
#define VELOCITY_TEMP(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)  // The ultrasonic velocity (cm/us) compensated by temperature

//define values for lidar and sonar
uint16_t wait = 100;
int16_t ft_lidar = 8;
int16_t bk_lidar = 9;
int16_t lt_lidar = 10;
int16_t rt_lidar = 11;
int16_t lidar_pins[4] = {ft_lidar,bk_lidar,lt_lidar,rt_lidar};
int16_t rt_trigechoPin = 3;
int16_t lt_trigechoPin = 4;
int16_t trig_EchoPin[2] = {rt_trigechoPin,lt_trigechoPin};

//define values for sensor distances
//In order: Forward, Back, Left, Right, ForwardRight, ForwardLeft
int Sensor_Distances[6] = {100000, 100000, 100000, 100000, 100000, 100000};
bool Object_in_Range = false;

// Helper Functions
//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT_EN] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT_EN] ++; //count the right wheel encoder interrupts
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
    lastSpeed[LEFT_EN] = encoder[LEFT_EN];                        //record the latest left speed value
    lastSpeed[RIGHT_EN] = encoder[RIGHT_EN];                      //record the latest right speed value
    accumTicks[LEFT_EN] = accumTicks[LEFT_EN] + encoder[LEFT_EN];    //record accumulated left ticks
    accumTicks[RIGHT_EN] = accumTicks[RIGHT_EN] + encoder[RIGHT_EN]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT_EN]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT_EN]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT_EN]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT_EN]);
    encoder[LEFT_EN] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT_EN] = 0;                         //clear the right encoder data buffer
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

/*This function, runToStop(), will run the robot until the target is achieved and
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
void moveR(float distance, int speed){
//  Serial.print("Right:  ");
  stepperRight.setCurrentPosition(0);//sets right motor position to 0
  int steps = 0;
  //First converts the distance we want to travel into number of steps
  if(distance>0){// when moving forward
    steps = convertFeetToSteps(distance, 0);
  } else{// when moving backward
    steps = convertFeetToSteps(distance, 1);
  }
//  Serial.println(steps);
  stepperRight.moveTo(steps);//move number of steps forward relative to current position
  stepperRight.setSpeed(speed);//set right motor speed
  stepperRight.runSpeedToPosition();//move right motor
}
void moveL(float distance, int speed){
//  Serial.print("Left:  ");
  stepperLeft.setCurrentPosition(0);//sets right motor position to 0
  int steps = 0;
  if(distance>0){// when moving forward
    steps = convertFeetToSteps(distance, 0);
  } else{// when moving backward
    steps = convertFeetToSteps(distance, 1);
  }
//  Serial.println(steps);
  stepperLeft.moveTo(steps);//move number of steps forward relative to current position
  stepperLeft.setSpeed(speed);//set right motor speed
  stepperLeft.runSpeedToPosition();//move right motor
}

/*
  The pivot function will use AccelStepper to move one motor forward and halt the other motor to turn the robot
  based on the angle given
*/
void pivot(int direction) {
//  Serial.println("Pivot");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  int speed = 2000; //Setting a constant for speed
  //Calculate the distance needed to travel for given rotation
  float angle = abs(direction);//ensures value is positive
  float percent_rot = (angle / 360); // returns a fraction of the angle out of 360 degrees
  float distance = 2 * Wheel_Dist * Pi * percent_rot; // calculates the distance the motor will travel where Wheel_Dist is the distance between two of the motors
  //Choose which direction we are turning
  if (direction < 0) {// when ccw
    moveR(distance, speed); //moves only right motor  
  }
  else { // when cw
    moveL(distance, speed); //moves only left motor
  }
  runToStop();//run until the robot reaches the target
}

/*
  The spin() function will use AccelSteppoer to rotate both motor clockwise or
  counter closewise based on the angle given
*/
void spin(int direction) {
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  int speed = 800; //Setting a constant for speed
  //Calculate the distance needed to travel for given rotation
  float angle = abs(direction);//ensures angle is positive
  float percent_rot = (angle / 360); //gives fraction of angle to full circle
  float distance = Wheel_Dist * Pi * percent_rot; //calculates circumference
  //Choose which direction we are turning
  if (direction > 0) {
    moveR(distance, speed); //set right motor cw
    moveL(-distance, speed); //set left motor ccw
  }
  else {
    moveR(-distance, speed); //set right motor ccw
    moveL(distance, speed); //set left motor cw
  }
//  Serial.println("Spin");
  runToStop();//run until the robot reaches the target
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
//  Serial.println("Forward");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  int steps = distance * 800; // converting distance value to steps
  moveR(distance, speed); //set right motor
  moveL(distance, speed); //set left motor
  steppers.runSpeedToPosition();
  //runToStop();//run until the robot reaches the target
}
/*
  The reverse() function will use AccelStepper to move the robot backwards for a certain distance inputted.
  The wheels will move in the same direction a  t the same speed*/
void reverse(float distance) {
//  Serial.println("Reverse");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  int speed = 1000; //set speed
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

void randomWander(){
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  int maxVal = 1000; //maximum value for random number
//  randomSeed(analogRead(0)); //generate a new random number each time called
  int randNumber = random(maxVal); //generate a random number up to the maximum value
  int randLR = random(0,1);
  // max distance = 5 feet
  // max angle = 180 degrees
  if((randNumber%2)==1){
    float distance = randNumber/200;  
    if(distance < 1){
      distance = 1;
    }
    forward(distance);
  } else if ((randNumber%2)==0){
    float diameter = randNumber/200;
    float angle;
    if(diameter < 1){
      diameter = 1;
    }
    if(randNumber > 360){
      angle = (randNumber-680);
      if (angle < 1 && angle > 0){
        angle = 90;
      } else if( angle < 0 && angle > -1){
        angle = -90;
      }
    }else {
      if(randLR == 0){
        angle = -randNumber;
      }else {
        angle = randNumber;
      }
    }
    turn(diameter,angle);
  }
  if(Object_in_Range == true) {
    Halt();
  }
  Serial.println("Wandering..");
}

// a struct to hold lidar data
struct sensor {
  // this can easily be extended to contain sonar data as well
  int front;
  int back;
  int left;
  int right;
  int sonarR;
  int sonarL;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right, sonarR, sonarL);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist;

// read_lidars is the function used to get lidar data to the M7
struct sensor read_lidars() {
  return dist;
}

struct sensor read_sonars(){
  return dist;
}

// reads a lidar given a pin
int read_lidar(int pin) {
  int16_t t = pulseIn(pin, HIGH);
  if (t == 0 || t > 1850)
    return 100000;
  int d = (t - 1000) * 3 / 40;
  if (d < 0) { d = 0; }
  return d;
}

int read_sonar(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);//Set the trig pin High
  delayMicroseconds(10);//Delay of 10 microseconds
  digitalWrite(pin, LOW);//Set the trig pin Low
  pinMode(pin, INPUT);//Set the pin to input mode
  
  int16_t t = pulseIn(pin, HIGH);
  int d = t * VELOCITY_TEMP(20) / 2.0;  //The distance can be calculated according to the flight time of ultrasonic wave,
                                        //and the ultrasonic sound speed can be compensated according to the actual ambient temperature
  return d;
}

void readSensors(){
  dist.front = read_lidar(8);
  dist.back = read_lidar(9);
  dist.left = read_lidar(10);
  dist.right = read_lidar(11);
  dist.sonarR = read_sonar(3);
  dist.sonarL = read_sonar(4);
//  //Reads the lidar sensors
  for (int i = 0;i<4;i++){
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.print(read_lidar(i));
    Sensor_Distances[i] = read_lidar(i+7);
//    Serial.print(" ");
    //delay(1);
  }
  Serial.println(Sensor_Distances[0]);
//  Serial.println();

    //Reads the left and right sonar
//  Sensor_Distances[4] = read_sonar(RIGHT_SON);
//  Sensor_Distances[5] = read_sonar(LEFT_SON);
//   Sensor_Distances[4] = read_sonar(3);
//   Sensor_Distances[4] = read_sonar(4);

}

// stores data to send to M7
void getData(){
  for (int i = 0;i<4;i++){
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.print(read_lidar(i));
    Sensor_Distances[i] = read_lidar(i+7);
//    Serial.print(" ");
    //delay(1);
  }

   Sensor_Distances[4] = read_sonar(3);
   Sensor_Distances[5] = read_sonar(4);
}

void Halt() {
  Object_in_Range = false;
//  readSensors();
  //Checks every sensor to see if there is an object in range
  for (int i = 0;i<6;i++) {
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.print(Sensor_Distances[i]);
    if (Sensor_Distances[i] == 0) {
      Sensor_Distances[i] = 2000;
    }
//    Serial.print(" ");
//    delay(10);
    if (Sensor_Distances[i] <= 10){
//      Serial.println("In Range");
      Object_in_Range = true;
    }
  }
  //Checks to see if there are any objects in range and if there aren't keep moving
  if(Object_in_Range == false) {
    Serial.println("Out of Range");
//    forward(0.7);
    randomWander();
  }
  else{
    Serial.println("Halt");
    stop();
  }
}

void Follow() {
  //Checks the values of the sensors and removes the ones out of range
  int active_range = 35;
  for (int i = 0;i<6;i++) {
    if (Sensor_Distances[i] == 2000) {
      Sensor_Distances[i] = 0;
    } else if (Sensor_Distances[i] > active_range) {
      Sensor_Distances[i] = 0;
    }
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.print(Sensor_Distances[i]);
//    Serial.print(" ");
  }
  //Calculate the angle the robot needs to move
  float Y_distance = Sensor_Distances[0] - Sensor_Distances[1] + Sensor_Distances[4]*cos(Pi/4) + Sensor_Distances[5]*cos(Pi/4);
  float X_distance =  Sensor_Distances[3] - Sensor_Distances[2] +  Sensor_Distances[4]*sin(Pi/4) - Sensor_Distances[5]*sin(Pi/4);
  //Check to see if robot is too close to object
  for (int i = 0;i<6;i++) {
    if (Sensor_Distances[i] <= 3 && Sensor_Distances[i] != 0) {
      return;
    }
  }
  float dist = sqrt(pow(X_distance,2) + pow(Y_distance,2)); //distance of point in feet
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
  goToAngle((int)-angle);//robot will turn at given angle
//  Serial.println();
//  Serial.println(dist);
  //Uses distance from object to calc speed
  if (dist <= 3) {
    return;
  }
  speed = dist/active_range*1000;
  if (speed == 0) {
    speed = 1000;
  }
  forward(dist/active_range*0.5);
  speed = 1000;
  
  delay(100);
  Serial.println("Following");
}

void Flee() {
  getData();
  //Checks the values of the sensors and removes the ones out of range
  int active_range = 35;
  for (int i = 0;i<6;i++) {
    if (Sensor_Distances[i] == 2000) {
      Sensor_Distances[i] = 0;
    } else if (Sensor_Distances[i] > active_range) {
      Sensor_Distances[i] = 0;
    }
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.print(Sensor_Distances[i]);
//    Serial.print(" ");
  }
  //Calculate the angle the robot needs to move
  float Y_distance = Sensor_Distances[0] - Sensor_Distances[1] + Sensor_Distances[4]*cos(Pi/4) + Sensor_Distances[5]*cos(Pi/4);
  float X_distance =  Sensor_Distances[3] - Sensor_Distances[2] +  Sensor_Distances[4]*sin(Pi/4) - Sensor_Distances[5]*sin(Pi/4);
  float dist = sqrt(pow(X_distance,2) + pow(Y_distance,2)); //distance of point in feet
  //Check to see if there no object in range
  if (dist == 0) {
    return;
  }
  float angle = ((atan2((float)X_distance, (float)Y_distance)) * 180 / Pi); //calculate angle of POI
  goToAngle((int)-angle);//robot will turn at given angle
  //Uses distance from object to calc speed
  speed = dist/active_range*1000;
  if (speed == 0) {
    speed = 1000;
  }
//  Serial.print("Distance: ");
//  Serial.println(active_range/dist*0.1);
  float distance = active_range/dist*0.1;
  if (distance >= 0.5) {
    distance = 0.5;
  }
  reverse(distance);
  speed = 1000;
  
  delay(100);
  Serial.println("Flee");
}

//set up the M4 to be the server for the sensors data
void setupM4() {
  Serial.begin(9600);
  delay(1000);
  RPC.bind("read_lidars", read_lidars);  // bind a method to return the lidar data all at once
  RPC.bind("read_sonars", read_sonars);
}

//poll the M4 to read the data
void loopM4() {
  // update the struct with current lidar data
//  dist.front = read_lidar(8);
//  dist.back = read_lidar(9);
//  dist.left = read_lidar(10);
//  dist.right = read_lidar(11);
//  dist.sonarR = read_sonar(3);
//  dist.sonarL = read_sonar(4);
  readSensors();

}

//set up the M7 to be the client and run the state machine
void setupM7() {
  // begin serial interface
//  Serial.begin(9600);
//  delay(1000);
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor
  randomSeed(analogRead(0)); //generate a new random number each time called

  for (int i = 0; i<numOfSens;i++){
    pinMode(lidar_pins[i],OUTPUT);
    delay(100);
  }

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  Serial.begin(baudrate);     //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
}

//read sensor data from M4 and write to M7
void loopM7() {
  // read lidar data from struct
  struct sensor dataLidar = RPC.call("read_lidars").as<struct sensor>();
  struct sensor dataSonar = RPC.call("read_sonars").as<struct sensor>();
//   print lidar data
//  Serial.print("Lidar Data: ");
//  Serial.print(dataLidar.front);
//  Serial.print(", ");
//  Serial.print(dataLidar.back);
//  Serial.print(", ");
//  Serial.print(dataLidar.left);
//  Serial.print(", ");
//  Serial.print(dataLidar.right);
////   print sonar data
//  Serial.print("  Sonar Data: ");
//  Serial.print(dataSonar.sonarL);
//  Serial.print(", ");
//  Serial.print(dataSonar.sonarR);
//  Serial.println();
  getData();
  Flee();
}

//setup function with infinite loops to send and receive sensor data between M4 and M7
void setup() {
  RPC.begin();
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    // if on M7 CPU, run M7 setup & loop
    setupM7();
    while (1) loopM7();
  } else {
    // if on M4 CPU, run M7 setup & loop
    setupM4();
    while (1) loopM4();
  }
}

// loop() is never called as setup() never returns
// this may need to be modified to run th estate machine.
// consider usingnamespace rtos Threads as seen in previous example
void loop() {}
