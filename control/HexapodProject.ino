#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "spiderLeg.h"
#include "Gait.h"
#include "trotForward.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

SpiderLeg leg1(&pwm, 0, 1, 2, 120, 130, 100);    // Front Left
SpiderLeg leg2(&pwm, 3, 4, 5, 140, 120, 100);     // Front Right
SpiderLeg leg3(&pwm, 6, 7, 8, 120, 130, 100);     // Rear Left
SpiderLeg leg4(&pwm, 9, 10, 11, 100, 110, 100);   // Rear Right

float stepLength = 50;
float stepHeight = 40;
int stepDelay = 8;
int xValue = 0;

Gait gait1(&leg1, stepLength, stepHeight, stepDelay);
Gait gait2(&leg2, stepLength, stepHeight, stepDelay);
Gait gait3(&leg3, stepLength, stepHeight, stepDelay);
Gait gait4(&leg4, stepLength, stepHeight, stepDelay);

trotForward spiderTrot(&gait1, &gait2, &gait3, &gait4);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685...");
  
  pwm.begin();
  pwm.setPWMFreq(60);  
  
  delay(10);
  
  Serial.println("Moving to neutral position...");
  
  // Move all legs to neutral position
  leg1.moveToPosition(0, 0, 0);
  leg2.moveToPosition(0, 0, 0);
  leg3.moveToPosition(0, 0, 0);
  leg4.moveToPosition(0, 0, 0);
  
  leg1.execute();
  leg2.execute();
  leg3.execute();
  leg4.execute();
  
  delay(1000);
  Serial.println("Ready to walk!");
}


void loop() {
  xValue = analogRead(A0);
  Serial.print("x = ");
  Serial.print(xValue);

  if (xValue > 700) {
    spiderTrot.shuffleForward(1);
  } else {  // stopping
    spiderTrot.prepareStance();
  }

  // spiderTrot.rotate(1, 1);

  // Make the quadruped walk forward in a trot gait
  // spiderTrot.continuousShuffle();
  // spiderTrot.alternatingWalk(2);
  // delay(500);
}