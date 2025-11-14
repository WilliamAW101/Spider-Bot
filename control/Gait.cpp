#include "Gait.h"
#include <Arduino.h>
#include <math.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Gait::Gait(SpiderLeg* leg, float stepLength, float stepHeight, int delayStep) {
  this->leg = leg;
  this->stepLength = stepLength;
  this->stepHeight = stepHeight;
  this->delayStep = delayStep;
}

// Move leg forward by stepLength
void Gait::stepForward() {
  const int frames = 30;  // number of steps (higher = smoother)
  
  // foot in air moving forward
  for (int i = 0; i < frames; i++) {
    float t = (float)i / (frames - 1); 
    float x = -stepLength / 2 + t * stepLength; // move from back to front
    float z = stepHeight * sin(t * M_PI); // lift smoothly
    
    leg->moveToPosition(x, z, 0);
    leg->execute();
    delay(delayStep);
  }
  
  // foot on ground moving backward 
  for (int i = 0; i < frames; i++) {
    float t = (float)i / (frames - 1); 
    float x = stepLength / 2 - t * stepLength; // move from front to back
    float z = 0; // foot on ground
    
    leg->moveToPosition(x, z, 0);
    leg->execute();
    delay(delayStep);
  }
}

void Gait::stepForwardInvert() {
  const int frames = 30;
  
  // foot in air moving forward 
  for (int i = 0; i < frames; i++) {
    float t = (float)i / (frames - 1);
    float x = -stepLength / 2 + t * stepLength;
    float z = stepHeight * sin(t * M_PI);
    
    // Invert X for opposite side legs
    leg->moveToPosition(-x, z, 0);
    leg->execute();
    delay(delayStep);
  }
  
  // Stance 
  for (int i = 0; i < frames; i++) {
    float t = (float)i / (frames - 1);
    float x = stepLength / 2 - t * stepLength;
    float z = 0;
    
    leg->moveToPosition(-x, z, 0);
    leg->execute();
    delay(delayStep);
  }
}


void Gait::testLegMotion() {
  stepForward();
  delay(1000);
}