#include "arm.h"
#include <Arduino.h>
#include <math.h>

arm::arm(Adafruit_PWMServoDriver* pwm, int gearPin) {
  this->pwm = pwm;
  this->servoGearPin = gearPin;
  this->position = 90; 
}

int arm::angleToPulse(int angle) {
  int pulseMin = 150;  
  int pulseMax = 600; 
  
  return map(angle, 0, 180, pulseMin, pulseMax);
}

void arm::open() {
  pwm->setPWM(servoGearPin, 0, angleToPulse(180)); 
}

void arm::close() {
  pwm->setPWM(servoGearPin, 0, angleToPulse(0));
}

void arm::init() {
  pwm->setPWM(servoGearPin, 0, angleToPulse(position));
}