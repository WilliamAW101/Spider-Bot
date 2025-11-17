#ifndef GAIT_H
#define GAIT_H
#include "spiderLeg.h"

class Gait {
public:
  Gait(SpiderLeg* leg, float stepLength, float stepHeight, int delayStep);
  
  void stepForward();
  void stepForwardInvert();
  void testLegMotion();
  
  friend class trotForward;
  friend class trotRotate;

private:
  SpiderLeg* leg;
  float stepLength;
  float stepHeight;
  int delayStep;
};

#endif