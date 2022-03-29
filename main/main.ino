// Obstacle Course Robot code

// Required Libraries
#include <Adafruit_MotorShield.h> // Library for Adafruit motor shield  
#include <NewPing.h> // Library for the Ultrasonic sensor
#include <Adafruit_ICM20X.h> // Libraries for the IMU
#include <Adafruit_ICM20948.h> // Libraries for the IMU
#include <Adafruit_Sensor.h> // Libraries for the IMU
#include <Wire.h> // Libraries for the IMU

// UltraSonic Sensors
// Setup Pins for UltraSonic Sensors
#define TRIGGER_PIN_F A3 // To Do: determine which pins these are on the board
#define ECHO_PIN_F    A2 // To Do: determine which pins these are on the board
#define TRIGGER_PIN_L A0 // To Do: determine which pins these are on the board
#define ECHO_PIN_L    A1 // To Do: determine which pins these are on the board
//#define TRIGGER_PIN_R A4 // To Do: determine which pins these are on the board
//#define ECHO_PIN_R    A5 // To Do: determine which pins these are on the board

// Maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 200 // To Do determine the max distance we need to work with

// Setup each of the UltraSonic sensors
NewPing sonarF(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonarL(TRIGGER_PIN_L, ECHO_PIN_L, MAX_DISTANCE);
// NewPing sonarR(TRIGGER_PIN_R, ECHO_PIN_R, MAX_DISTANCE); // maybe not used ?!

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


// Motors
// initialize motors
// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *motorBR = AFMS.getMotor(3);
Adafruit_DCMotor *motorFR = AFMS.getMotor(4);
Adafruit_DCMotor *motorFL = AFMS.getMotor(1);
Adafruit_DCMotor *motorBL = AFMS.getMotor(2);

// set initial speed to this
int MOTOR_SPEED_FORWARD = 125; // 255 is max, To Do: determine the speed
int MOTOR_SPEED_TURNING = 50; // slower for turning, 255 is max, To Do: determine the speed


// IMU Sensor 
// Definition
Adafruit_ICM20948 imu;


// Helper functions

void stopMotors() {
  motorBR->run(RELEASE);
  motorFR->run(RELEASE);
  motorBL->run(RELEASE);
  motorFL->run(RELEASE); 
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

//  for (int speedSet = 0; speedSet < MOTOR_SPEED_FORWARD; speedSet +=2) // slowly bring the speed up 
//                                                          // to reduce stress on motors and battery
//  {
//   motorBR->setSpeed(speedSet);
//   motorFR->setSpeed(speedSet);
//   motorBL->setSpeed(speedSet);
//   motorFL->setSpeed(speedSet);
//   delay(5);
//  }
}

float getYaw(){ 
    // Source:
    // https://students.iitk.ac.in/roboclub/2017/12/21/Beginners-Guide-to-IMU.html
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp, &mag);

    float accelX = accel.acceleration.x;
    float accelY = accel.acceleration.y;
    float accelZ = accel.acceleration.z;

    float magReadX = mag.magnetic.x;
    float magReadY = mag.magnetic.y;
    float magReadZ = mag.magnetic.z;

    // unit changed to in radian
    float pitch = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ));
    float roll = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ));

    float mag_x = magReadX*cos(pitch) + magReadY*sin(roll)*sin(pitch) + magReadZ*cos(roll)*sin(pitch);
    float mag_y = magReadY * cos(roll) - magReadZ * sin(roll);
    float yaw = 180 * atan2(-mag_y,mag_x)/M_PI;

    return yaw; 
}

float front_dist_arr[] = {999,999,999,999,999,999,999,999,999,999}; // Large value place holders
int front_i = 0;
float getFrontDist() { // in cm
// use running average to ensure stability
  delay(50);
  front_dist_arr[++front_i] = sonarF.ping_cm();

  float total = 0;
  for (int j=0; j<10; ++j) {
      total += front_dist_arr[j];
  }
  Serial.print("Ping Front: ");
  Serial.print(total/10); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  return  total / 10;
}

float left_dist_arr[] = {999,999,999,999,999,999,999,999,999,999}; // Large value place holders
int left_i = 0;
float getLeftDist() { // in cm
// use running average to ensure stability
  delay(50); 
  left_dist_arr[++left_i] = sonarF.ping_cm();

  float total = 0;
  for (int j=0; j<10; ++j) {
      total += left_dist_arr[j];
  }
  Serial.print("Ping Left: ");
  Serial.print(total/10); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  return  total / 10;
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
  float angleMargin = 10; // degree, To Do: determine the margin
  float distMargin = 5; // left dist, To Do: determine the margin
  float angleTurnAdjust = 15; // deg, To Do: determine the angle
  float blindDuration = 3000; // ms, To Do: determine the duration
  float angle = getYaw(); // this angle is the "straight", recorded at each start after turning 90 deg 
  
  moveForward();
  while (getFrontDist() > minFrontDist) {
    if (getLeftDist() < leftDist - distMargin) {
      turnToAngle((int(angle)+15) % 360); // assumption, right turn increases yaw
      moveForward();
      delay(blindDuration);
      stopMotors();
      turnToAngle(angle);
      moveForward();
    } else if (getLeftDist() > leftDist + distMargin) {
      turnToAngle((int(angle)-15) % 360); // assumption, right turn increases yaw
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
  
// testing functions

// test 1 Average speed test
void speedTest(int stoppingDistance){
  moveForward();
  while(getFrontDist() > stoppingDistance){}
  stopMotors();
}

// test 2 Turn test
void turnTest(int turnAngle){
  delay(100);
  turnToAngle(turnAngle);
  delay(100);
}

// test 3 Detection and Reaction test
void detectAndReactTest(int reactionDistance, int test3Time){
  int frontDist = getFrontDist();
  int leftDist = getLeftDist();

    if (frontDist <= reactionDistance){
      Serial.println("Front Detected Turning");
      turnRight();
      delay(test3Time);
     }
     else if (getLeftDist() <= reactionDistance) {
       Serial.println("Left Detected Turning");
       turnRight();
       delay(test3Time);
     }
  }
  stopMotors();
}

// function to convert distance count to approximate tile count with a margin of error

// possible helper function to convert distances to tile numbers if needed
//int convertDistToTile(){
//  }

// preliminary spiral path algorithm

// test version of path traversal algorithm


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

  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G); // 4 G force
  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS); // 1000 degrees/s
  
}

void blindTurn(int turnTime){
  turnRight();
  delay(turnTime);
  }

void loop() {
  // put your main code here, to run repeatedly:

  // testing turn each motors slowly for 5 s

//  motorBR->setSpeed(MOTOR_SPEED_TURNING);
//  motorFR->setSpeed(MOTOR_SPEED_TURNING);
//  motorBL->setSpeed(MOTOR_SPEED_TURNING);
//  motorFL->setSpeed(MOTOR_SPEED_TURNING);

//  motorBR->run(FORWARD);
//  delay(5000);
//  motorFR->run(FORWARD);
//  delay(5000);
//  motorBL->run(FORWARD);
//  delay(5000);
//  motorFL->run(FORWARD);
//  delay(5000);

  // March 25th design check testing
  
  speedTest(5); // stop 5 cm infront of obtruction at end of path

  turnTest(85); // turn to 85 degrees, assuming there is delay in wheels which would get us to 90

  detectAndReactTest(5, 1000); // detect hand/object at least 5 cm away and wait at least 1 s between detections

  // path algorithm 
  // no for loop used for easier debugging separtely one by one later 

  float FRONT_MARGIN = 5; // min dist to edge before turning
  float SIDE_MARGIN = 5;
  float BLOCK_WIDTH = 30;

  // first loop
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

// may be better to implement the above as a series of switch statements
// that way we don't have to restart from the beginning if something goes wrong with one

// get initial states

int front = 0;
int left = 0;

front = getFrontDist();
left = getFrontDist();

float distanceArray[2] {front, left};

// need to add function to determine state
// switch state machine incomplete
//switch(state){
//  case loop1;
//      moveForwardUntil(FRONT_MARGIN, SIDE_MARGIN);
//      turnRight90();
//    break;
//  case loop1end;
//      moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//      SIDE_MARGIN + BLOCK_WIDTH * 1);
//    turnRight90();
//    break;
//  case loop2;
//    moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 1,
//    SIDE_MARGIN + BLOCK_WIDTH * 1);
//    turnRight90();
//    break;
//  case loop2end;
//    moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//    SIDE_MARGIN + BLOCK_WIDTH * 1);
//    break;
//  case loop3;
//    moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//    SIDE_MARGIN + BLOCK_WIDTH * 2);
//    turnRight90();
//    break;
//  case loop3end;
//    moveForwardUntil(FRONT_MARGIN + BLOCK_WIDTH * 2,
//    SIDE_MARGIN + BLOCK_WIDTH * 2);
//    break;
//  }

  // testSpiralPath();
  

}
