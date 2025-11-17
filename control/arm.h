#ifndef ARM_H
#define ARM_H

#include <Adafruit_PWMServoDriver.h>

class arm {
  public:
    arm(Adafruit_PWMServoDriver* pwm, int gearPin);
    void open();
    void close();
    
  private:
    Adafruit_PWMServoDriver* pwm;
    int servoGearPin;
    int position;  // Initial position
    
    // Helper function to convert angle to pulse width
    int angleToPulse(int angle);
};

#endif