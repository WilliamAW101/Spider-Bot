#ifndef TROT_ROTATE_H
#define TROT_ROTATE_H

#include "trotForward.h"
#include "Gait.h"

class trotRotate : public trotForward {
private:
  float tibiaGrip;  // Tibia grip angle in degrees
  float rearLiftReduction;    

public:
  trotRotate(Gait* frontLeft, Gait* frontRight, Gait* rearLeft, Gait* rearRight, int restDelay);
  
  // 1 = clockwise, -1 = counterclockwise
  void rotate(int cycles, int direction);
  
  void rotateClockwise(int cycles);
  void rotateCounterClockwise(int cycles);
  
  // Quick turn by approximate degrees
  void quickTurn(int direction, int degrees);
  
  // Configuration 
  void setTibiaGrip(float degrees);              
  void setRearLiftReduction(float factor);       
};

#endif