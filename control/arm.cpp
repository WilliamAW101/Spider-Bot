#include "arm.h"
#include <Arduino.h>
#include <math.h>

arm::arm(Adafruit_PWMServoDriver* pwm, int gearPin) {
  this->pwm = pwm;
  this->servoGearPin = gearPin;
  this->position = 90;  // Default initial position (adjust as needed)
}

int arm::angleToPulse(int angle) {
  // Convert angle (0-180) to pulse width (typically 150-600 for servos)
  // Adjust these values based on your servo specs
  int pulseMin = 150;  // Minimum pulse length (0 degrees)
  int pulseMax = 600;  // Maximum pulse length (180 degrees)
  
  return map(angle, 0, 180, pulseMin, pulseMax);
}

void arm::open() {
  // Implement your open logic here
  pwm->setPWM(servoGearPin, 0, angleToPulse(180));  // Example
}

void arm::close() {
  // Implement your close logic here
  pwm->setPWM(servoGearPin, 0, angleToPulse(0));  // Example
}

void arm::init() {
  pwm->setPWM(servoGearPin, 0, angleToPulse(position));
}