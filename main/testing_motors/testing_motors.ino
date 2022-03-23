// file to help test the motors

#include <AFMotor.h> // Library for Adafruit motor shield  

// initialize motors

// Motors
// initialize motors
AF_DCMotor motorBR(1, MOTOR12_1KHZ); // To Do: determine what PWM frequencies to use for motors
AF_DCMotor motorFR(2, MOTOR12_1KHZ); // To Do: determine what PWM frequencies to use for motors
AF_DCMotor motorFL(3, MOTOR34_1KHZ); // To Do: determine what PWM frequencies to use for motors
AF_DCMotor motorBL(4, MOTOR34_1KHZ); // To Do: determine what PWM frequencies to use for motors



int MOTOR_SPEED_SLOW = 20; // slow motor speed for testing
int MOTOR_SPEED_TURNING = 50;
int MOTOR_SPEED_FORWARD = 120; // test value

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

//  for (int speedSet = 0; speedSet < MOTOR_SPEED_FORWARD; speedSet +=2) // slowly bring the speed up 
//                                                          // to reduce stress on motors and battery
//  {
//   motorBR.setSpeed(speedSet);
//   motorFR.setSpeed(speedSet);
//   motorBL.setSpeed(speedSet);
//   motorFL.setSpeed(speedSet);
//   delay(5);
//  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // opens serial port and sets data rate to 115200 bps

  // setup IMU
//  if (imu.begin_I2C()) {
//    Serial.println("IMU Found!");
//  }
//  else {
//     // IMU not found, infinitely looping to indicate that something is wrong
//    while(1){
//      Serial.println("IMU not found.");
//      }
//  }
//
//  imu.setAccelRange(ICM20948_ACCEL_RANGE_8_G); // 8 G force
//  imu.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS); // 1000 degrees/s
//  
}

void loop() {
  // put your main code here, to run repeatedly:

  // testing turn each motors slowly for 5 s

  motorBR.setSpeed(MOTOR_SPEED_SLOW);
  motorFR.setSpeed(MOTOR_SPEED_SLOW);
  motorBL.setSpeed(MOTOR_SPEED_SLOW);
  motorFL.setSpeed(MOTOR_SPEED_SLOW);

  motorBR.run(FORWARD);
  delay(5000);
  motorFR.run(FORWARD);
  delay(5000);
  motorBL.run(FORWARD);
  delay(5000);
  motorFL.run(FORWARD);
  delay(5000);
}
