#include "spiderLeg.h"
#include <Arduino.h>
#include <math.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// PCA9685 pulse width constants (microseconds)
#define SERVOMIN  150  // ~0 degrees 
#define SERVOMAX  600  // ~180 degrees 

SpiderLeg::SpiderLeg(Adafruit_PWMServoDriver* pwm,
                     int coxaChannel, int femurChannel, int tibiaChannel,
                     int coxaOffset, int femurOffset, int tibiaOffset) {
  this->pwm = pwm;
  this->coxaChannel = coxaChannel;
  this->femurChannel = femurChannel;
  this->tibiaChannel = tibiaChannel;
  
  // Store neutral offsets
  this->coxaOffset = coxaOffset;
  this->femurOffset = femurOffset;
  this->tibiaOffset = tibiaOffset;
  
  // Default limits (tune to your robot)
  coxaMin = 0;   coxaMax = 180;
  femurMin = 0;  femurMax = 180;
  tibiaMin = 0;  tibiaMax = 180;
  
  // Initialize angles to 0 (relative to neutral)
  coxaAngle = 0;
  femurAngle = 0;
  tibiaAngle = 0;
}

int SpiderLeg::angleToPulse(int angle) {
  // Map 0-180 degrees to SERVOMIN-SERVOMAX pulse width
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

bool SpiderLeg::moveToPosition(float x, float y, float z) {
  // Simple mapping: treat x, y, z as relative servo angles
  // x controls coxa (horizontal rotation)
  // y controls femur (up/down)  
  // z controls tibia (foot angle)
  
  // Direct angle control - no complex IK
  moveTo(x, y, z);
  return true;
}

void SpiderLeg::moveTo(float coxaAngle, float femurAngle, float tibiaAngle) {
  // Store relative angles (will be offset when executing)
  this->coxaAngle = coxaAngle;
  this->femurAngle = femurAngle;
  this->tibiaAngle = tibiaAngle;
}

void SpiderLeg::moveBy(float coxaDelta, float femurDelta, float tibiaDelta) {
  moveTo(coxaAngle + coxaDelta, femurAngle + femurDelta, tibiaAngle + tibiaDelta);
}

void SpiderLeg::execute() {
  // Apply neutral offsets to get absolute servo positions
  int coxaPos  = constrain((int)(coxaAngle + coxaOffset), coxaMin, coxaMax);
  int femurPos = constrain((int)(femurAngle + femurOffset), femurMin, femurMax);
  int tibiaPos = constrain((int)(tibiaAngle + tibiaOffset), tibiaMin, tibiaMax);
  
  // Send PWM signals via Driver
  pwm->setPWM(coxaChannel, 0, angleToPulse(coxaPos));
  pwm->setPWM(femurChannel, 0, angleToPulse(femurPos));
  pwm->setPWM(tibiaChannel, 0, angleToPulse(tibiaPos));
}

float SpiderLeg::getCoxaAngle() { 
  return coxaAngle; 
}

float SpiderLeg::getFemurAngle() { 
  return femurAngle; 
}

float SpiderLeg::getTibiaAngle() { 
  return tibiaAngle; 
}