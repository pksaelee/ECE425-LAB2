///*
// * Final Project PS2 Controller Program 
// * Created on 01/30/2024, last edited on  01/30/2024
// *
// * PURPOSE
// * This program is designed to allow control over our 2-wheel robot with gripper using a PS2 controller.
// * It utilizes the PS2X library written by Bill Porter (github link at https://github.com/madsci1016/Arduino-PS2X).
// * Every loop, it reads the controller, then selects the appropriate action based on the pressed buttons
// * and joystick values of the controller.  The analog sticks dictate the direction of movement: forward or
// * backward, spinning left or right, or turning left or right while moving forward.  Meanwhile, the R1 and L1
// * buttons cause the gripper to open and close in steps of 2 degrees per loop.
// *
// * HARDWARE
// * -2 Continuous Servo Motors
// * -1 Bluetooth PS2 Controller with Dongle
// *
// * VARIABLES
// * Servo Motors
// * Upper and Lower Bounds for Motor Movement, Analog Stick Positions
// * Pins for PS2 USB Dongle
// * PS2 Option variables
// * PS2 Controller Object
// *
// * FUNCTIONS
// * straight()
// * spin()
// * turnRight()
// * turnLeft()
// *
// */
//
//// Library includes for Servos and PS2 controller
//#include <Arduino.h>//include for PlatformIO Ide
//#include <AccelStepper.h>//include the stepper motor library
//#include <MultiStepper.h>//include multiple stepper motor library
////#include <Servo.h>
//#include <PS2X_lib.h>
//
////// Pin definintions
////#define leftMotor 13
////#define rightMotor 12
////#define gripperPin 11
////define motor pin numbers
//#define stepperEnable 48    //stepper enable pin on stepStick
//#define rtStepPin 50 //right stepper motor step pin
//#define rtDirPin 51  // right stepper motor direction pin
//#define ltStepPin 52 //left stepper motor step pin
//#define ltDirPin 53  //left stepper motor direction pin
//
//AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
//AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
//MultiStepper steppers;//create instance to control multiple steppers at the same time
//
//#define stepperEnTrue false //variable for enabling stepper motor
//#define stepperEnFalse true //variable for disabling stepper motor
//#define max_speed 1500 //maximum stepper motor speed
//#define max_accel 10000 //maximum motor acceleration
//
//int pauseTime = 2500;   //time before robot moves
//int stepTime = 500;     //delay time between high and low on step pin
//int wait_time = 1000;   //delay for printing data
//
//// Boundary setting
//#define SERVO_MAX 1700
//#define SERVO_MIN 1300
//#define SERVO_STOP 1500
//#define STICK_MAX 255
//#define STICK_MIN 0
//
//// Pin defines for PS2X library
//#define PS2_DAT 8 // 12
//#define PS2_CMD 7 // 11
//#define PS2_SEL 6 // 10
//#define PS2_CLK 9 // 13
//
//#define pressures false
//#define rumble false
//
////int redLED = A3;
////int grnLED = A0;
////state LEDs connections
//#define redLED 7            //red LED for displaying states
//#define grnLED 5          //green LED for displaying states
//#define ylwLED 6            //yellow LED for displaying states
//#define enableLED 13        //stepper enabled LED
//int leds[3] = {5, 6, 7};    //array of LED pin numbers
//
//// Creates PS2 controller, creates type and error variables for initialization
//PS2X controller;
//
//int error = 0;
//byte type = 0;
//
////int angle = GRIPPER_OPEN;
//
//// Creates servo motors
////Servo gripperServo;
////Servo servoLeft;
////Servo servoRight;
//
////stop: 1500; clockwise: 1300, counterclockwise: 1700
//
//void init_stepper() {
//  pinMode(rtStepPin, OUTPUT);//sets pin as output
//  pinMode(rtDirPin, OUTPUT);//sets pin as output
//  pinMode(ltStepPin, OUTPUT);//sets pin as output
//  pinMode(ltDirPin, OUTPUT);//sets pin as output
//  pinMode(stepperEnable, OUTPUT);//sets pin as output
//  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
//  pinMode(enableLED, OUTPUT);//set enable LED as output
//  digitalWrite(enableLED, LOW);//turn off enable LED
//  pinMode(redLED, OUTPUT);//set red LED as output
//  pinMode(grnLED, OUTPUT);//set green LED as output
//  pinMode(ylwLED, OUTPUT);//set yellow LED as output
//  digitalWrite(redLED, HIGH);//turn on red LED
//  digitalWrite(ylwLED, HIGH);//turn on yellow LED
//  digitalWrite(grnLED, HIGH);//turn on green LED
//  delay(pauseTime / 5); //wait 0.5 seconds
//  digitalWrite(redLED, LOW);//turn off red LED
//  digitalWrite(ylwLED, LOW);//turn off yellow LED
//  digitalWrite(grnLED, LOW);//turn off green LED
//
//  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
//  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
//  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
//  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
//  steppers.addStepper(stepperRight);//add right motor to MultiStepper
//  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
//  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
//  digitalWrite(enableLED, HIGH);//turn on enable LED
//}
//
//void setup() {
//  // Setup code, runs once
//  // Attach servos to pins
////  gripperServo.attach(gripperPin);
////  servoLeft.attach(leftMotor);
////  servoRight.attach(rightMotor);
//  init_stepper();
//
//  pinMode(redLED, OUTPUT);
//  pinMode(grnLED, OUTPUT);
//
//  // Begin serial monitor
//  Serial.begin(57600);
//
//  delay(300);
//
//  
//  // sets variable for initialization check
//  int error = controller.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
//  while(error != 0) {
//    int error = controller.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
//    digitalWrite(grnLED, LOW);
//    digitalWrite(redLED, HIGH);
//  }
//
//
//  // Prints results of initialzation process
//  if(error == 0) {
//    Serial.println("Controller config SUCCESS");
//  } else {
//    Serial.println("Controller config FAILED");
//  }
//
//
//  // Outputs whether or not controller type is read correctly
//  type = controller.readType();
//  switch(type) {
//    case 0:
//      Serial.println("Unknown controller type found");
//      break;
//    case 3:
//      Serial.println("Wireless DualShock controller found");
//      break;
//    default:
//      Serial.println("Wrong controller type found");
//      break;
//  }
//
//  // Set motors to starting positions and speeds
////  gripperServo.write(angle);
////  servoRight.writeMicroseconds(SERVO_STOP);
////  servoLeft.writeMicroseconds(SERVO_STOP);
//
//}
//
//void loop() {
//  // Main loop, runs forever
//
//  //Break out of function if no controller is found
//  if(error == 1) {
//    return;
//  }
//
//  // Read controller
//  controller.read_gamepad(false, 0);
//
//  digitalWrite(grnLED, HIGH);
//  digitalWrite(redLED, LOW);
//  
//  // Trigger IR LED when square is pressed (only once per press)
//  if(controller.NewButtonState()) {
//    if(controller.Button(PSB_SQUARE)) {
//      Serial.println("IR LED triggered:");
//    }
//  }
//
//  // Open or close gripper if the corresponding bumper is pressed, incrementally
//  if(controller.Button(PSB_R1)) {
//    Serial.println("Closing gripper:");
////    closeGripper();
//  }
//  else if(controller.Button(PSB_L1)) {
//    Serial.println("Opening gripper:");
////    openGripper();
//  }
//
//  // Selects one mode of movement depending on stick values from initial read, or stops the motors
//  if(controller.Analog(PSS_LY) <= 122) {
//    int stickValue = controller.Analog(PSS_LY);
//    Serial.println("Moving forward (degree set by future code)");
////    straight(stickValue);
//  }
//  else if(controller.Analog(PSS_LY) >= 132) {
//    int stickValue = controller.Analog(PSS_LY);
//    Serial.println("Moving backward (degree set by future code)");
////    straight(stickValue);
//  }
//  else if(controller.Analog(PSS_LX) <= 122) {
//    int stickValue = controller.Analog(PSS_LX);
//    Serial.println("Spinning left (degree set by future code)");
////    spin(stickValue);
//  }
//  else if(controller.Analog(PSS_LX) >= 132) {
//    int stickValue = controller.Analog(PSS_LX);
//    Serial.println("Spinning right (degree set by future code)");
////    spin(stickValue);
//  }
//  else if(controller.Analog(PSS_RX) <= 122) {
//    int stickValue = controller.Analog(PSS_RX);
//    Serial.println("Turning left (degree set by future code)");
////    turnLeft(stickValue);
//  }
//  else if(controller.Analog(PSS_RX) >= 132) {
//    int stickValue = controller.Analog(PSS_RX);
//    Serial.println("Turning right (degree set by future code)");
////    turnRight(stickValue);
//  }
//  else {
////    stopMotors();
//    Serial.println("Stopped");
//  }
//
//
//  delay(20);
//  
//}
//
////void closeGripper(){
////  // Closes the gripper one degree at a time
////  if(angle < GRIPPER_CLOSED){
////    gripperServo.write(angle+=2);
////  } 
////}
////
////void openGripper(){
////  // Opens the gripper one degree at a time
////  if(angle > GRIPPER_OPEN){
////    gripperServo.write(angle-=2);
////  } 
////}
////void spin(int stick){
////  // Make both motors move in opposite directions at the same speed, proportional to the input
//////  servoLeft.writeMicroseconds(servoConvert(stick));
//////  servoRight.writeMicroseconds(servoConvert(stick));
////  digitalWrite(redLED, HIGH);//turn on red LED
////  digitalWrite(grnLED, HIGH);//turn on green LED
////  digitalWrite(ylwLED, LOW);//turn off yellow LED
////  
////}
////
////void straight(int stick){
////  // Make both motors move in the same direction, proportional to the input
////  servoLeft.writeMicroseconds(servoConvert(STICK_MAX - stick));
////  servoRight.writeMicroseconds(servoConvert(stick));
////  Serial.print("Stick: ");  Serial.print(stick);  Serial.print("Left: ");  Serial.print(servoConvert(STICK_MAX - stick));  Serial.print("Right: ");  Serial.print(servoConvert(stick));
////}
////
////void turnLeft(int stick){
////  // Make the right motor move forward at max speed and the left move forward proportional to input
////  servoLeft.writeMicroseconds(servoConvert((STICK_MAX / 2) + stick));
////  servoRight.writeMicroseconds(servoConvert(STICK_MIN));
////}
////
////void turnRight(int stick){
////  // Make the left motor move forward at max speed and the right move forward proportional to input
////  servoLeft.writeMicroseconds(servoConvert(STICK_MAX));
////  servoRight.writeMicroseconds(servoConvert(stick / 2));
////}
////
////void stopMotors() {
////  // Stop both the motors
////  servoLeft.writeMicroseconds(SERVO_STOP);
////  servoRight.writeMicroseconds(SERVO_STOP);
////}
////
////int servoConvert(int input){
////  // Does stick-to-servo conversions for the right motor
////  int convert = map(input, STICK_MIN, STICK_MAX, SERVO_MIN, SERVO_MAX);
////  return convert;
////}
