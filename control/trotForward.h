#ifndef TROTFORWARD_H
#define TROTFORWARD_H

#include "Gait.h"

class trotForward {
public:
  Gait* FL; // Front Left leg gait
  Gait* FR; // Front Right leg gait
  Gait* RL; // Rear Left leg gait
  Gait* RR; // Rear Right leg gait

  int restDelay;  // Pause between leg cycles
  bool walking; // Used for continuous modes

  trotForward(Gait* frontLeft, Gait* frontRight, Gait* rearLeft, Gait* rearRight, int restDelay = 50);

  // Primary gait
  void shuffleForward(int cycles);  // Smooth diagonal shuffle gait (replaces side shuffle)
  void alternatingWalk(int cycles); // this crap sucked

  // Continuous versions
  void continuousShuffle(); // Loop shuffle until stopped

  // rotation
  void trotForward::rotate(int cycles, int direction);

  
  void stop();  // Immediately stop continuous gait
  void smoothStop();  // Gracefully stop by returning to stance
  void prepareStance(); // Reset all legs to neutral/stance position
  
};

#endif
