// Obstacle Course Robot code

// Required Libraries

#include <AFMotor.h> // Library for Adafruit motor shield  
#include <NewPing.h> // Library for the Ultrasonic sensor
#include <Adafruit_ICM20X.h> // Libraries for the IMU
#include <Adafruit_ICM20948.h> // Libraries for the IMU
#include <Adafruit_Sensor.h> // Libraries for the IMU

// UltraSonic Sensors
// Setup Pins for UltraSonic Sensors
#define TRIGGER_PIN_F A0 // To Do: determine which pins these are on the board
#define ECHO_PIN_F    A1 // To Do: determine which pins these are on the board
#define TRIGGER_PIN_L A2 // To Do: determine which pins these are on the board
#define ECHO_PIN_L    A3 // To Do: determine which pins these are on the board
#define TRIGGER_PIN_R A4 // To Do: determine which pins these are on the board
#define ECHO_PIN_R    A5 // To Do: determine which pins these are on the board

// Maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 300 // To Do determine the max distance we need to work with

// Setup each of the UltraSonic senssors
NewPing sonarF(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonarL(TRIGGER_PIN_L, ECHO_PIN_L, MAX_DISTANCE);
// NewPing sonarR(TRIGGER_PIN_R, ECHO_PIN_R, MAX_DISTANCE); // maybe not used ?!


// Motors
// initialize motors
AF_DCMotor motorBR(1, MOTOR12_1KHZ); // To Do: determine what PWM frequencies to use for motors
AF_DCMotor motorFR(2, MOTOR12_1KHZ); // To Do: determine what PWM frequencies to use for motors
AF_DCMotor motorFL(3, MOTOR34_1KHZ); // To Do: determine what PWM frequencies to use for motors
AF_DCMotor motorBL(4, MOTOR34_1KHZ); // To Do: determine what PWM frequencies to use for motors

// set initial speed to this
int MOTOR_SPEED_FORWARD = 125; // 255 is max, To Do: determine the speed
int MOTOR_SPEED_TURNING = 50; // slower for turning, 255 is max, To Do: determine the speed


// IMU Sensor 
// Definition
Adafruit_ICM20948 imu;


// Helper functions

void stopMotors() {
  motorBR.run(RELEASE);
  motorFR.run(RELEASE);
  motorBL.run(RELEASE);
  motorFL.run(RELEASE); 
}

void turnRight() {
  // set motor speed
  motorBR.setSpeed(MOTOR_SPEED_TURNING);
  motorFR.setSpeed(MOTOR_SPEED_TURNING);
  motorBL.setSpeed(MOTOR_SPEED_TURNING);
  motorFL.setSpeed(MOTOR_SPEED_TURNING);

  // turning right
  motorBR.run(BACKWARD);
  motorFR.run(BACKWARD);
  motorBL.run(FORWARD);
  motorFL.run(FORWARD);
}

void turnLeft() {
  // set motor speed
  motorBR.setSpeed(MOTOR_SPEED_TURNING);
  motorFR.setSpeed(MOTOR_SPEED_TURNING);
  motorBL.setSpeed(MOTOR_SPEED_TURNING);
  motorFL.setSpeed(MOTOR_SPEED_TURNING);

  // turning left
  motorBR.run(FORWARD);
  motorFR.run(FORWARD);
  motorBL.run(BACKWARD);
  motorFL.run(BACKWARD);
}

void moveForward() {
  // set motor speed
  motorBR.setSpeed(MOTOR_SPEED_FORWARD);
  motorFR.setSpeed(MOTOR_SPEED_FORWARD);
  motorBL.setSpeed(MOTOR_SPEED_FORWARD);
  motorFL.setSpeed(MOTOR_SPEED_FORWARD);

  // moving forward
  motorBR.run(FORWARD);
  motorFR.run(FORWARD);
  motorBL.run(FORWARD);
  motorFL.run(FORWARD);
}

//float getYaw(){ 
//  // need to test first and see if the function works
//  // back-up idea: using magnetometer to find yaw?
//  return 0; 
//}

float getFrontDist() { // in cm
  delay(50); 
  return sonarF.ping_cm(); 
}

float getLeftDist() { // in cm
  delay(50); 
  return sonarL.ping_cm();
}

void turnToAngle(int ang) { // angle should be in [0, 360) degree
  // decides whether left or right turn and turn to that desired angle
  int angleMargin = 3; // in degree, To Do: determine the margin
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
  int angleMargin = 10; // degree, To Do: determine the margin
  int distMargin = 5; // left dist, To Do: determine the margin
  int angleTurnAdjust = 15; // deg, To Do: determine the angle
  int blindDuration = 3000; // ms, To Do: determine the duration
  int angle = getYaw(); // this angle is the "straight", recorded at each start after turning 90 deg 
  
  moveForward();
  while (getFrontDist() > minFrontDist) {
    if (getLeftDist() < leftDist - distMargin) {
      turnToAngle((angle+15) % 360); // assumption, right turn increases yaw
      moveForward();
      delay(blindDuration);
      stopMotors();
      turnToAngle(angle);
      moveForward();
    } else if (getLeftDist() > leftDist + distMargin) {
      turnToAngle((angle-15) % 360); // assumption, right turn increases yaw
      moveForward();
      delay(blindDuration);
      stopMotors();
      turnToAngle(angle);
      moveForward();
    }

    if (abs(angle-getYaw()) > angleMargin) { // margin of error
      turnToAngle(angle);
    }
  }
  stopMotors();  
}

void turnRight90() {
  int angle = getYaw(); // this angle is the "straight", recorded at the start
  turnToAngle((angle + 90) % 360);
}

// testing functions

// test 1 Average speed test
void speedTest(int stoppingDistance){
   if (getFrontDist()> stoppingDistance){
    moveForward();
    }
    else {
      stopMotors();
      }
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // opens serial port and sets data rate to 115200 bps

  // setup IMU
  if (imu.begin_I2C()) {
    Serial.println("IMU Found!");
  }
  else {
    Serial.println("IMU not found.");
  }

  imu.setAccelRange(ICM20948_ACCEL_RANGE_8_G); // 8 G force
  imu.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS); // 1000 degrees/s
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // testing turn each motors slowly for 5 s

  motorBR.setSpeed(MOTOR_SPEED_TURNING);
  motorFR.setSpeed(MOTOR_SPEED_TURNING);
  motorBL.setSpeed(MOTOR_SPEED_TURNING);
  motorFL.setSpeed(MOTOR_SPEED_TURNING);

  motorBR.run(FORWARD);
  delay(5000);
  motorFR.run(FORWARD);
  delay(5000);
  motorBL.run(FORWARD);
  delay(5000);
  motorFL.run(FORWARD);
  delay(5000);

}
