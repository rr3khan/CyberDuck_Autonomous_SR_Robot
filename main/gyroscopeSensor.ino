// Global variables to help with rotation

// deprecated using gyroscope to get yaw

//float previousTime = 0;
//float currentAngle = 0;
//float newAngle = 0;
//
//// Get yaw angle from gyro z axis, returns the change in angle
//
//float getYaw(){
//   // Get a new normalized sensor event
//  sensors_event_t accel;
//  sensors_event_t gyro;
//  sensors_event_t mag;
//  sensors_event_t temp;
//  imu.getEvent(&accel, &gyro, &temp, &mag);
//
//  float gyroX = gyro.gyro.x;
//  float gyroY = gyro.gyro.y;
//  float gyroZ = gyro.gyro.z;
//
//  Serial.print("\n\tGyroscope X: ");
//  Serial.print(gyroX);
//  Serial.print(" \tY: ");
//  Serial.print(gyroY);
//  Serial.print(" \tZ: ");
//  Serial.print(gyroZ);
//  Serial.println(" radians/s ");
//  Serial.println();
//
//  float currentAngularVelocity = gyroZ; // read Z value from gyroscope
//  
//  // start checking when there is movement
//  if (abs(gyroZ) >= 0.05){
//    float currentTime = millis();
//
//    float deltaTime = (currentTime - previousTime)/1000; // get time change in s
//    float angleChange = 180/M_PI * (currentAngularVelocity * deltaTime); // find angle change in degrees
//
//    currentAngle += angleChange; // keep track of angle accumulation
//    previousTime = currentTime; // update previous time
//    }
//  return currentAngle;
//  }
//
//// Determine when robot has moved 90 degrees
//
//void check90(){
//  newAngle = 0;
//  currentAngle = 0;
//  float angleToCheck = 0;
//
//  while(abs(angleToCheck) < 75){ // Wheels may still be moving a bit, so add a buffer
//                                 // will need to verufy this during testing
//    angleToCheck = getYaw();
//    }
//  }
