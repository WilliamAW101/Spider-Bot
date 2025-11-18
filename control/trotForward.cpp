#include "trotForward.h"
#include <Arduino.h>
#include <math.h>

// Smooth ease in/out for natural motion
static inline float easeInOutQuad(float t) {
  return t < 0.5f ? 2.0f * t * t : 1.0f - powf(-2.0f * t + 2.0f, 2.0f) / 2.0f; // yeah wat
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

trotForward::trotForward(Gait* frontLeft, Gait* frontRight, Gait* rearLeft, Gait* rearRight, int restDelay) {
  this->FL = frontLeft;
  this->FR = frontRight;
  this->RL = rearLeft;
  this->RR = rearRight;
  this->restDelay = restDelay;
  this->walking = false;
}

void trotForward::shuffleForward(int cycles) {
  const int frames = 80;
  const float stanceRatio = 0.4f;
  
  for (int cycle = 0; cycle < cycles; cycle++) {
    // Phase 1: FL + RR swing, FR + RL stance
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      // Swing legs: FL + RR
      float swingX = -FL->stepLength / 2 + ease * FL->stepLength;
      float swingZ = (t < stanceRatio) ? 0.0f
                      : FL->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
      
      FL->leg->moveToPosition(swingX, swingZ, 0);
      RR->leg->moveToPosition(-swingX, -swingZ, 0);
      
      // FR + RL push backward
      float stanceEase = easeInOutQuad(1.0f - t);
      float stanceX = stanceEase * (FL->stepLength / 4) - (FL->stepLength / 8);
      
      FR->leg->moveToPosition(stanceX, 0, 0);
      RL->leg->moveToPosition(stanceX, 0, 0);
      
      FL->leg->execute();
      FR->leg->execute();
      RL->leg->execute();
      RR->leg->execute();
      
      delay(FL->delayStep);
    }
    delay(restDelay / 2);
    
    // FR + RL swing, FL + RR stance
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      // Swing legs: FR + RL
      float swingX = -FR->stepLength / 2 + ease * FR->stepLength;
      float swingZ = (t < stanceRatio) ? 0.0f
                      : FR->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
      
      FR->leg->moveToPosition(-swingX, swingZ, 0);
      RL->leg->moveToPosition(swingX, -swingZ, 0);
      
      // Stance legs: FL + RR push backward
      float stanceEase = easeInOutQuad(1.0f - t);
      float stanceX = stanceEase * (FR->stepLength / 4) - (FR->stepLength / 8);
      
      FL->leg->moveToPosition(stanceX, 0, 0);
      RR->leg->moveToPosition(stanceX, 0, 0);
      
      FL->leg->execute();
      FR->leg->execute();
      RL->leg->execute();
      RR->leg->execute();
      
      delay(FR->delayStep);
    }
    delay(restDelay / 2);
  }
}

void trotForward::alternatingWalk(int cycles) {
  const int frames = 30;
  
  for (int cycle = 0; cycle < cycles; cycle++) {
    
    // Move Front Left 
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      float swingX = -FL->stepLength / 2 + ease * FL->stepLength;
      float swingZ = FL->stepHeight * sinf(t * M_PI);
      
      FL->leg->moveToPosition(swingX, swingZ, 0);
      FL->leg->execute();
      delay(FL->delayStep);
    }
    delay(restDelay);
    
    // Move Rear Right  
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      float swingX = -RR->stepLength / 2 + ease * RR->stepLength;
      float swingZ = RR->stepHeight * sinf(t * M_PI);
      
      RR->leg->moveToPosition(-swingX, -swingZ, 0);
      RR->leg->execute();
      delay(RR->delayStep);
    }
    delay(restDelay);
    
    // Move Front Right 
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      float swingX = -FR->stepLength / 2 + ease * FR->stepLength;
      float swingZ = FR->stepHeight * sinf(t * M_PI);
      
      FR->leg->moveToPosition(-swingX, swingZ, 0);
      FR->leg->execute();
      delay(FR->delayStep);
    }
    delay(restDelay);
    
    // Move Rear Left 
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      float swingX = -RL->stepLength / 2 + ease * RL->stepLength;
      float swingZ = RL->stepHeight * sinf(t * M_PI);
      
      RL->leg->moveToPosition(swingX, -swingZ, 0);
      RL->leg->execute();
      delay(RL->delayStep);
    }
    delay(restDelay);
    
    // all legs push back together 
    const int stanceFrames = 25;
    for (int f = 0; f < stanceFrames; f++) {
      float t = (float)f / (stanceFrames - 1);
      float ease = easeInOutQuad(t);
      
      // Push all legs backward to advance body forward
      float pushX = (FL->stepLength / 2) * (1.0f - ease);
      
      FL->leg->moveToPosition(pushX, 0, 0);
      FR->leg->moveToPosition(-pushX, 0, 0);
      RL->leg->moveToPosition(pushX, 0, 0);
      RR->leg->moveToPosition(-pushX, 0, 0);
      
      FL->leg->execute();
      FR->leg->execute();
      RL->leg->execute();
      RR->leg->execute();
      
      delay(FL->delayStep);
    }
  }
}

void trotForward::rotate(int cycles, int direction) {
  // 1 = clockwise,
  // -1 = counterclockwise
  const int frames = 80;
  const float stanceRatio = 0.4f;
  
  // Rotation radius multiplier (adjust based on robot geometry) 
  float rotationFactor = (float)direction;
  
  for (int cycle = 0; cycle < cycles; cycle++) {
    
    // FL + RR swing
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      // Swing legs in arc for rotation
      // FL moves forward/backward based on direction
      float swingX_FL = rotationFactor * (-FL->stepLength / 2 + ease * FL->stepLength);
      float swingZ = (t < stanceRatio) ? 0.0f
                      : FL->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
      
      // RR moves opposite direction for rotation
      float swingX_RR = -rotationFactor * (-RR->stepLength / 2 + ease * RR->stepLength);
      
      FL->leg->moveToPosition(-swingX_FL, swingZ, 0);
      RR->leg->moveToPosition(swingX_RR, -swingZ, 0);
      
      // Stance legs: FR + RL push in rotation direction
      float stanceEase = easeInOutQuad(1.0f - t);
      float stanceX_FR = -rotationFactor * (stanceEase * (FL->stepLength / 4) - (FL->stepLength / 8));
      float stanceX_RL = rotationFactor * (stanceEase * (FL->stepLength / 4) - (FL->stepLength / 8));
      
      FR->leg->moveToPosition(stanceX_FR, 0, 0);
      RL->leg->moveToPosition(stanceX_RL, 0, 0);
      
      FL->leg->execute();
      FR->leg->execute();
      RL->leg->execute();
      RR->leg->execute();
      
      delay(FL->delayStep);
    }
    delay(restDelay / 2);

    // FR + RL swing
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      // Swing legs in arc for rotation
      float swingX_FR = -rotationFactor * (-FR->stepLength / 2 + ease * FR->stepLength);
      float swingZ = (t < stanceRatio) ? 0.0f
                      : FR->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
      
      float swingX_RL = rotationFactor * (-RL->stepLength / 2 + ease * RL->stepLength);
      
      FR->leg->moveToPosition(swingX_FR, swingZ, 0);
      RL->leg->moveToPosition(swingX_RL, -swingZ, 0);
      
      // FL + RR push in rotation direction
      float stanceEase = easeInOutQuad(1.0f - t);
      float stanceX_FL = rotationFactor * (stanceEase * (FR->stepLength / 4) - (FR->stepLength / 8));
      float stanceX_RR = -rotationFactor * (stanceEase * (FR->stepLength / 4) - (FR->stepLength / 8));
      
      FL->leg->moveToPosition(stanceX_FL, 0, 0);
      RR->leg->moveToPosition(stanceX_RR, 0, 0);
      
      FL->leg->execute();
      FR->leg->execute();
      RL->leg->execute();
      RR->leg->execute();
      
      delay(FR->delayStep);
    }
    delay(restDelay / 2);
  }
}

void trotForward::continuousShuffle() {
  walking = true;
  while (walking) {
    shuffleForward(1);
  }
}

void trotForward::stop() {
  walking = false;
}

void trotForward::smoothStop() {
  walking = false;
  prepareStance();
}

void trotForward::prepareStance() {
  // Return all legs to neutral stance position
  FL->leg->moveToPosition(0, 0, 0);
  FR->leg->moveToPosition(0, 0, 0);
  RL->leg->moveToPosition(0, 0, 0);
  RR->leg->moveToPosition(0, 0, 0);
  
  FL->leg->execute();
  FR->leg->execute();
  RL->leg->execute();
  RR->leg->execute();
}