#ifndef SPIDERLEG_H
#define SPIDERLEG_H
#include <Adafruit_PWMServoDriver.h>

class SpiderLeg {
private:
  Adafruit_PWMServoDriver* pwm;
  
  // Servo channel numbers on Driver
  int coxaChannel;
  int femurChannel;
  int tibiaChannel;
  
  // Current angles (in degrees)
  float coxaAngle;
  float femurAngle;
  float tibiaAngle;
  
  // Neutral position offsets
  int coxaOffset;
  int femurOffset;
  int tibiaOffset;
  
  // Safety limits 
  int coxaMin, coxaMax;
  int femurMin, femurMax;
  int tibiaMin, tibiaMax;
  
  // Convert angle to PWM pulse width for Driver
  int angleToPulse(int angle);
  
public:
  SpiderLeg(Adafruit_PWMServoDriver* pwm, 
            int coxaChannel, int femurChannel, int tibiaChannel,
            int coxaOffset, int femurOffset, int tibiaOffset);
  
  // Simple relative movement
  bool moveToPosition(float x, float y, float z);
  
  // Direct angle control
  void moveTo(float coxaAngle, float femurAngle, float tibiaAngle);
  void moveBy(float coxaDelta, float femurDelta, float tibiaDelta);
  
  void execute();
  
  float getCoxaAngle();
  float getFemurAngle();
  float getTibiaAngle();
};

#endif