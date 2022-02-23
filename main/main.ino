// Obstacle Course Robot code

// Required Libraries

#include <AFMotor.h> // Library for Adafruit motor shield  
#include <NewPing.h> // Library for the Ultrasonic sensor

// initialize motors
// to do determine what PWM frequencies to use for motors

AF_DCMotor motorBR(1, MOTOR12_1KHZ); // BR = Back Right motor
AF_DCMotor motorFR(2, MOTOR12_1KHZ); // FR = Front Right motor
AF_DCMotor motorFL(3, MOTOR34_1KHZ);
AF_DCMotor motorBL(4, MOTOR34_1KHZ);

// Helper functions

void turnRight() {
  motorBR.run(BACKWARD);
  motorFR.run(BACKWARD);
  motorBL.run(FORWARD);
  motorFL.run(FORWARD);
}



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
