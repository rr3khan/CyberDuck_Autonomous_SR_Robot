// Obstacle Course Robot code

// Required Libraries
#include <Adafruit_MotorShield.h> // Library for Adafruit motor shield  
#include <NewPing.h> // Library for the Ultrasonic sensor
#include <Adafruit_ICM20X.h> // Libraries for the IMU
#include <Adafruit_ICM20948.h> // Libraries for the IMU
#include <Adafruit_Sensor.h> // Libraries for the IMU
#include <Wire.h> // Libraries for the IMU
#include "MPU9250.h"

// UltraSonic Sensors
// Setup Pins for UltraSonic Sensors
#define TRIGGER_PIN_F A3
#define ECHO_PIN_F    A2
#define TRIGGER_PIN_L A0
#define ECHO_PIN_L    A1
//#define TRIGGER_PIN_R A4 
//#define ECHO_PIN_R    A5 

// Maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 300

// Setup each of the UltraSonic sensors
NewPing sonarF(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonarL(TRIGGER_PIN_L, ECHO_PIN_L, MAX_DISTANCE);
// NewPing sonarR(TRIGGER_PIN_R, ECHO_PIN_R, MAX_DISTANCE);


// Setup Pins for Start Button
#define START_BUTTON_PIN 22


// Motors
// initialize motors

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motorFL = AFMS.getMotor(1);
Adafruit_DCMotor *motorBL = AFMS.getMotor(2);
Adafruit_DCMotor *motorBR = AFMS.getMotor(3);
Adafruit_DCMotor *motorFR = AFMS.getMotor(4);

// set initial speed to this
int MOTOR_SPEED_FORWARD = 25; // 255 is max, 20: can turn, 35: for test 1
int MOTOR_SPEED_TURNING = 70; // slower for turning, 255 is max, 45: can turn, 50: for test 2 & 3


// IMU Sensor 
// Definition
Adafruit_ICM20948 imu;


// Helper functions

void stopMotors() {
  motorBR->run(RELEASE);
  motorFR->run(RELEASE);
  motorBL->run(RELEASE);
  motorFL->run(RELEASE); 

  motorBR->setSpeed(0);
  motorFR->setSpeed(0);
  motorBL->setSpeed(0);
  motorFL->setSpeed(0);
}

void turnRight() {
  // set motor speed
  motorBR->setSpeed(MOTOR_SPEED_TURNING);
  motorFR->setSpeed(MOTOR_SPEED_TURNING);
  motorBL->setSpeed(MOTOR_SPEED_TURNING);
  motorFL->setSpeed(MOTOR_SPEED_TURNING);

  // turning right
  motorBR->run(BACKWARD);
  motorFR->run(BACKWARD);
  motorBL->run(FORWARD);
  motorFL->run(FORWARD);
}

void turnLeft() {
  // set motor speed
  motorBR->setSpeed(MOTOR_SPEED_TURNING);
  motorFR->setSpeed(MOTOR_SPEED_TURNING);
  motorBL->setSpeed(MOTOR_SPEED_TURNING);
  motorFL->setSpeed(MOTOR_SPEED_TURNING);

  // turning left
  motorBR->run(FORWARD);
  motorFR->run(FORWARD);
  motorBL->run(BACKWARD);
  motorFL->run(BACKWARD);
}

void moveForward() {
  // set motor speed
  motorBR->setSpeed(MOTOR_SPEED_FORWARD);
  motorFR->setSpeed(MOTOR_SPEED_FORWARD);
  motorBL->setSpeed(MOTOR_SPEED_FORWARD);
  motorFL->setSpeed(MOTOR_SPEED_FORWARD);

  // moving forward
  motorBR->run(FORWARD);
  motorFR->run(FORWARD);
  motorBL->run(FORWARD);
  motorFL->run(FORWARD);
}

void moveBackward() {
// set motor speed
  motorBR->setSpeed(MOTOR_SPEED_FORWARD);
  motorFR->setSpeed(MOTOR_SPEED_FORWARD);
  motorBL->setSpeed(MOTOR_SPEED_FORWARD);
  motorFL->setSpeed(MOTOR_SPEED_FORWARD);

  // moving forward
  motorBR->run(BACKWARD);
  motorFR->run(BACKWARD);
  motorBL->run(BACKWARD);
  motorFL->run(BACKWARD);

}

float previousTime = 0;
float currentAngle = 0;
float newAngle = 0;
// Get yaw angle from gyro z axis, returns the change in angle
float getYaw(){
  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);

  float gyroX = gyro.gyro.x; // in radians/s
  float gyroY = gyro.gyro.y; // in radians/s
  float gyroZ = gyro.gyro.z; // in radians/s

  float currentAngularVelocity = gyroZ; // read Z value from gyroscope

// start checking when there is movement
//  if (abs(gyroZ) >= 0.05){
    float currentTime = millis();

    float deltaTime = (currentTime - previousTime)/1000; // get time change in s
    float angleChange = 180/M_PI * (currentAngularVelocity * deltaTime); // find angle change in degrees

    currentAngle += angleChange; // keep track of angle accumulation
    previousTime = currentTime; // update previous time
//  }

  return -currentAngle;
}

float front_dist_arr[] = {MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE}; // Large value place holders
int front_i = 0;
bool front_dist_init = false;

float getFrontDist() { // in cm
// use running average to ensure stability
  delay(50);
  front_i = (front_i + 1) % 3;
  front_dist_arr[front_i] = sonarF.ping_cm();
  if(front_dist_arr[front_i] == 0){
    front_dist_arr[front_i] = 300; // for the case when front ultrasonic detects 0 because it's too far 
  }

  float total = 0;
  for (int j=0; j<3; ++j) {
      total += front_dist_arr[j];
  }
  return  total / 3;
}

float left_dist_arr[] = {MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE}; // Large value place holders
int left_i = 0;
float getLeftDist() { // in cm
// use running average to ensure stability
  delay(50); 
  left_i = (left_i + 1) % 3;
  left_dist_arr[left_i] = sonarL.ping_cm();
  if(front_dist_arr[front_i] == 0){
    front_dist_arr[front_i] = 300; // for the case when left ultrasonic detects 0 because it's too far 
  }

  float total = 0;
  for (int j=0; j<3; ++j) {
      total += left_dist_arr[j];
  }
  return  total / 3;
}

void turnToAngle(int ang) { // angle should be in [0, 360) degree
  // decides whether left or right turn and turn to that desired angle
  int angleMargin = 5; // due to braking distance, in degree, To Do: determine the margin
  int angdiff = ang - getYaw(); // To Do: verify assumption: turn right increases yaw
  
  if (angdiff < 0) {
    angdiff += 360; // should be always in [0, 360)
  }
  
  // choose direction to turn to minimize turning movement
  if (angdiff < 180) {
    turnRight();
  } else {
    turnLeft();
  }

  // stop when the yaw gets close to the target angle (margin)
  while (abs(getYaw() - ang) > angleMargin && abs(getYaw() - ang) < 360 - angleMargin) {}
  stopMotors();
}

void turnLeft45() {
  float angle = getYaw(); // this angle is the "straight", recorded at the start
  turnToAngle((int(angle) - 45) % 360); //turnToAngle((int(initialAngle) + 90) % 360) // turn right is +90
}

void turnRight90() {
  float angle = getYaw(); // this angle is the "straight", recorded at the start
  turnToAngle((int(angle) + 90) % 360); //turnToAngle((int(initialAngle) + 90) % 360) // turn right is +90
}

void moveForwardUntil(float minFrontDist) { // case 1: moving straight forward does not need left 
  float angleMargin = 10; // degree, To Do: determine the margin
  float angle = getYaw(); // this angle is the "straight", recorded at each start after turning 90 deg 
  
  moveForward();
  while (getFrontDist() > minFrontDist) {
    if (abs(angle-getYaw()) > angleMargin) { // margin of error
      turnToAngle(angle);
    }
  }
  stopMotors();
}

void moveForwardUntil(float minFrontDist, float leftDist) { // case 2: moving straight forward needs left
  float angleMargin = 25; // degree, To Do: determine the margin
  float distMargin = 4; // left dist, To Do: determine the margin
  float angleTurnAdjust = 15; // deg, To Do: determine the angle
  float blindDuration = 300; // ms, To Do: determine the duration
  float angle = getYaw(); // this angle is the "straight", recorded at each start after turning 90 deg 
  
  moveForward();
  while (getFrontDist() > minFrontDist) { 
    if (getLeftDist() > leftDist + distMargin) { // turning Left == far to the wall
      turnToAngle(int(getYaw()-angleTurnAdjust) % 360); // assumption, right turn increases yaw
      moveForward();
      delay(blindDuration);
      turnToAngle(int(getYaw()+angleTurnAdjust) % 360);
      moveForward();
    }
    if (getLeftDist() < leftDist - distMargin) { // turning Right == close to the wall
      turnToAngle(int(getYaw()+angleTurnAdjust) % 360); // assumption, right turn increases yaw
      moveForward();
      delay(blindDuration+200);
      turnToAngle(int(getYaw()-12) % 360); 
      moveForward();
    } 
  }
    stopMotors();  
}
  
// testing functions

// test 1 Average speed test
void speedTest(int stoppingDistance){
  moveForward();
  while(getFrontDist() > stoppingDistance){
    Serial.println(getFrontDist());
  }
  stopMotors();
}

// test 2 Turn test
void turnTest(int angle){
  delay(100);
  turnToAngle(angle);
  delay(100);
}

// test 3 Detection and Reaction test
void detectAndReactTest(int reactionDistance, int test3Time){
  delay(100);
  while (1) {
    if (getFrontDist() <= reactionDistance && getFrontDist() != 0){
      Serial.println("Front Detected Turning");
      turnRight();
      delay(test3Time);
      break;
    }
    else if (getLeftDist() <= reactionDistance) {
      Serial.println("Left Detected Turning");
      turnLeft();
      delay(test3Time);
      break;
    }
  }
  stopMotors();
}

// test 2&3 
void detectTurnAndReactTest(int reactionDistance){
  delay(100);
  while (1) {
    if (getFrontDist() <= reactionDistance && getFrontDist() != 0){
      Serial.println("Front Detected Turning");
      turnRight90();
      break;
    }
    else if (getLeftDist() <= reactionDistance  && getLeftDist() != 0) {
      Serial.println("Left Detected Turning");
      turnRight90();
      break;
    }
  }
  stopMotors();
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // opens serial port and sets data rate to 115200 bps

  // check motor shield
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  
  // setup IMU
  if (imu.begin_I2C()) {
    Serial.println("IMU Found!");
  }
  else {
     // IMU not found, infinitely looping to indicate that something is wrong
    while(1){
      Serial.println("IMU not found.");
      }
  }

  imu.setAccelRange(ICM20948_ACCEL_RANGE_8_G); // 8 G force
  imu.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS); // 1000 degrees/s

  delay(2000); // for safety
}

void loop() {
// put your main code here, to run repeatedly:


// March 25th design check testing

//// Test 1: Average speed test
//  speedTest(30); // stop 20 cm in front of obtruction at end of path
//  while(1);

//// Test 2: Turn test
//  turnTest(90);
//  while(1);
  
//// Test 3: Detection and Reaction test
//  detectAndReactTest(10, 500); // 10 cm to detect and 0.5 second turning 

//// Test 2&3: 
//  detectTurnAndReactTest(10); // 10 cm to detect (front) and 90 degrees turning 


//________________________________________________________________________________

//  moveForwardUntil(50, 20); // 50 cm to detect (front), keep 20 cm distance from the left to the wall, speed 30, 70
//  while(1);
  
//  moveForwardUntil(20, 10);
//  while(1);

//________________________________________________________________________________


//  // path algorithm 
//  // no for loop used for easier debugging separtely one by one later 
//
//  float FRONT_MARGIN = 40; // min dist to edge before turning
//  float SIDE_MARGIN = 19;
//  float BLOCK_WIDTH = 30;
//
//  // first loop
//  moveForwardUntil(FRONT_MARGIN, SIDE_MARGIN);
//  turnRight90();
//  
//  moveForwardUntil(FRONT_MARGIN, SIDE_MARGIN);
//  turnRight90();
//
//  moveForwardUntil(FRONT_MARGIN, SIDE_MARGIN);
//  turnRight90();
//
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 1, SIDE_MARGIN);
//  turnRight90();
//  
//  // second loop
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 1,
//                   SIDE_MARGIN + BLOCK_WIDTH * 1);
//  turnRight90();
//
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 1,
//                   SIDE_MARGIN + BLOCK_WIDTH * 1);
//  turnRight90();
//
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 1,
//                   SIDE_MARGIN + BLOCK_WIDTH * 1);
//  turnRight90();
//
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//                   SIDE_MARGIN + BLOCK_WIDTH * 1);
//  turnRight90();
//
//  // third loop
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//                   SIDE_MARGIN + BLOCK_WIDTH * 2);
//  turnRight90();
//
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//                   SIDE_MARGIN + BLOCK_WIDTH * 2);
//  turnRight90();
//
//  moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//                   SIDE_MARGIN + BLOCK_WIDTH * 2);
}
