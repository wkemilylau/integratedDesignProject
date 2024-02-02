void loop() {
  // in C++, size of char = number of letters + 1
  routePtr = (blockNumber - 1) * 4 + !pickup * 2 + blockType;
  if (routePtr < 0) {                   // from start to block 1
    routefollow("LR", 2);
    identifyblock();
    liftblock();
  } else if (routePtr >= 14) {          // return to finish
    junctionrotation('R');
    int numberOfJunctions = sizeOfRoutes[routePtr];
    routefollow(routes[routePtr], numberOfJunctions);
    while (1) {                         // stop after finish
      stopmoving();
    }    
  } else {
    if (!pickup || blockNumber == 1 || blockNumber == 2) {           // u turn to leave outpost and residential area
      junctionrotation('R');                                         
    }
    int numberOfJunctions = sizeOfRoutes[routePtr];
    routefollow(routes[routePtr], numberOfJunctions);

    if ((blockNumber == 2 || blockNumber == 3) && !pickup) {         // arrived open area
      // findandapproachblock(); 
      identifyblock();
      liftblock();
      // returntoline();
    } else if (pickup) {                                             // arrived outpost
      release();
    } else if ((blockNumber == 0 || blockNumber == 1) && !pickup) {  // arrived residential area                                                         
      identifyblock();
      liftblock();
    }
  }

  
}