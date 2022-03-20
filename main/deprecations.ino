//  // function to convert distance count to approximate tile count with a margin of error
//
//// possible helper function to convert distances to tile numbers if needed
////int convertDistToTile(){
////  }
//
//// preliminary spiral path algorithm
//
//// test version of path traversal algorithm
//
//void testSpiralPath(){
//  // 1 tile is about 30 cm long
//  // width of chassis is 130 mm => half  = 65 mm
//  // assume robot is placed about centre of tile
//  // hence rough distance from wall is about 150 - 65 = 85 mm
//  // hence use about 9 cm as 0 title distance for left
//
//  // getFrontDist()
//
//  // initialize algorithm variables
//
//  int turnCount = 0;
//  int rotation = 1;
//  
//  // turn condition variables
//  int forwardCondition = 9;
//  int leftCondition = 9;
//  delay(100);
//  // get forward distance
//  int frontDist = getFrontDist();
//  int leftDist = getLeftDist();
// 
//  while(turnCount < 10){
//
//    // check distances
//    
//    frontDist = getFrontDist();
//    leftDist = getLeftDist();
//    
//    // first 3 turn are against the wall hence left distance not needed
//    while (turnCount < 3){
//
//      // check distances
//       frontDist = getFrontDist();
//       leftDist = getLeftDist();
//
//      // move forward until min forward distance 
//      moveForwardUntil(forwardCondition);
//      Serial.println("Stopped, turn count: ");
//      Serial.println(turnCount);
//
//      // should be stopped here so update variables
//      turnCount++;
//      rotation = ceil(turnCount/4);
//
//      // turn 
//      turnRight90();  
//      }
//
//      // check if need to update turning condition
//
//      if (turnCount < 4*rotation - 1){
//        // Do nothing no need to update anything
//        }
//      else if (turnCount == 4*rotation - 1){
//        // update Front condition by one tile
//        forwardCondition += 30;
//        } 
//       else {
//        // turns > 4*rotation - 1 case hence update left condition by 1 title
//        leftCondition += 30;
//        }
//
//       // check distances
//       frontDist = getFrontDist();
//       leftDist = getLeftDist();
//
//      // move forward until min forward distance 
//      moveForwardUntil(forwardCondition, leftCondition);
//      Serial.println("Stopped, turn count: ");
//      Serial.println(turnCount);
//
//      // should be stopped here so update variables
//      turnCount++;
//      rotation = ceil(turnCount/4);
//
//      // turn 
//      turnRight90();  
//    }
//
//    // after 10th turn we are right infront of final tile
//    // hence move up slowly and then idle doing nothing
//    moveForwardUntil(69); // about 2 tiles and 9 cm
//    // still need to decide how long it takes to move up 1 tile
//    //delay(50);
//    stopMotors();
//    // idle we are done with the course:
//    while(true){};
//  }
