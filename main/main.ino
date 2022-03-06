// Obstacle Course Robot code

// Required Libraries

#include <AFMotor.h> // Library for Adafruit motor shield  
#include <NewPing.h> // Library for the Ultrasonic sensor

// Libraries for the IMU

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// Setup Pins for UltraSonic Sensors

// To do determine which pins these are on the board

#define TRIGGER_PIN_F A0
#define ECHO_PIN_F    A1
#define TRIGGER_PIN_L A2
#define ECHO_PIN_L    A3
#define TRIGGER_PIN_R A4 
#define ECHO_PIN_R    A5

// To do determine the max distance we need to work with

#define MAX_DISTANCE 300

// Setup each of the UltraSonic sensors

NewPing sonarF(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonarL(TRIGGER_PIN_L, ECHO_PIN_L, MAX_DISTANCE);
NewPing sonarR(TRIGGER_PIN_R, ECHO_PIN_R, MAX_DISTANCE);

// initialize motors
// to do determine what PWM frequencies to use for motors

AF_DCMotor motorBR(1, MOTOR12_1KHZ); // BR = Back Right motor
AF_DCMotor motorFR(2, MOTOR12_1KHZ); // FR = Front Right motor
AF_DCMotor motorFL(3, MOTOR34_1KHZ);
AF_DCMotor motorBL(4, MOTOR34_1KHZ);

// IMU Sensor Definition
Adafruit_ICM20948 imu;

// Helper functions

void turnRight() {
  motorBR.run(BACKWARD);
  motorFR.run(BACKWARD);
  motorBL.run(FORWARD);
  motorFL.run(FORWARD);
}

void moveForward() {
  motorBR.run(FORWARD);
  motorFR.run(FORWARD);
  motorBL.run(FORWARD);
  motorFL.run(FORWARD);
}

// Reading values from the UltraSonic sensor



void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

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

}
