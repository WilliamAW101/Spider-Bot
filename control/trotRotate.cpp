#include "trotRotate.h"
#include <Arduino.h>
#include <math.h>

// Smooth ease in/out for natural motion
static inline float easeInOutQuad(float t) {
  return t < 0.5f ? 2.0f * t * t : 1.0f - powf(-2.0f * t + 2.0f, 2.0f) / 2.0f;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

trotRotate::trotRotate(Gait* frontLeft, Gait* frontRight, Gait* rearLeft, Gait* rearRight, int restDelay)
  : trotForward(frontLeft, frontRight, rearLeft, rearRight, restDelay) {
  this->tibiaGrip = 15.0f;
  this->rearLiftReduction = 1.0f;
}

void trotRotate::setTibiaGrip(float degrees) {
  this->tibiaGrip = degrees;
}

void trotRotate::setRearLiftReduction(float factor) {
  this->rearLiftReduction = constrain(factor, 0.3f, 1.0f);
}

void trotRotate::rotate(int cycles, int direction) {
  const int frames = 80;
  const float stanceRatio = 0.3f;
  const float tibiaGrip = 15.0f;
  
  for (int cycle = 0; cycle < cycles; cycle++) {
    
    // FL + RR swing (diagonal pair)
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      // For rotation: both legs move in same rotational direction
      float swingProgress = -0.5f + ease;  
      float swingX_FL = direction * swingProgress * FL->stepLength;  // Both go same direction for rotation
      float swingX_RR = direction * swingProgress * RR->stepLength;
      
      // Swing height and tibia position
      float swingZ, tibiaZ_swing;
      if (t < stanceRatio) {
        swingZ = 0.0f;
        tibiaZ_swing = tibiaGrip;
      } else {
        swingZ = FL->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
        tibiaZ_swing = 0.0f;
      }
      
      FL->leg->moveToPosition(swingX_FL, swingZ, tibiaZ_swing);
      RR->leg->moveToPosition(swingX_RR, -swingZ * rearLiftReduction, tibiaZ_swing);
      
      // Stance legs: FR and RL push opposite direction
      float stanceProgress = 0.5f - ease;
      float stanceX_FR = -direction * stanceProgress * FR->stepLength;
      float stanceX_RL = -direction * stanceProgress * RL->stepLength;
      
      float tibiaZ_stance = tibiaGrip * (1.0f - t);
      
      FR->leg->moveToPosition(stanceX_FR, 0, tibiaZ_stance);
      RL->leg->moveToPosition(stanceX_RL, 0, tibiaZ_stance);
      
      FL->leg->execute();
      FR->leg->execute();
      RL->leg->execute();
      RR->leg->execute();
      
      delay(FL->delayStep);
    }
    delay(restDelay / 2);
    
    // FR + RL swing (diagonal pair)
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      float swingProgress = -0.5f + ease;
      float swingX_FR = direction * swingProgress * FR->stepLength;
      float swingX_RL = direction * swingProgress * RL->stepLength;
      
      // Swing height and tibia position
      float swingZ, tibiaZ_swing;
      if (t < stanceRatio) {
        swingZ = 0.0f;
        tibiaZ_swing = tibiaGrip;
      } else {
        swingZ = FR->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
        tibiaZ_swing = 0.0f;
      }
      
      FR->leg->moveToPosition(swingX_FR, swingZ, tibiaZ_swing);
      RL->leg->moveToPosition(swingX_RL, -swingZ * rearLiftReduction, tibiaZ_swing);
      
      // Stance legs push opposite direction
      float stanceProgress = 0.5f - ease;
      float stanceX_FL = -direction * stanceProgress * FL->stepLength;
      float stanceX_RR = -direction * stanceProgress * RR->stepLength;
      
      float tibiaZ_stance = tibiaGrip * (1.0f - t);
      
      FL->leg->moveToPosition(stanceX_FL, 0, tibiaZ_stance);
      RR->leg->moveToPosition(stanceX_RR, 0, tibiaZ_stance);
      
      FL->leg->execute();
      FR->leg->execute();
      RL->leg->execute();
      RR->leg->execute();
      
      delay(FR->delayStep);
    }
    delay(restDelay / 2);
  }
}

void trotRotate::rotateClockwise(int cycles) {
  rotate(cycles, 1);
}

void trotRotate::rotateCounterClockwise(int cycles) {
  rotate(cycles, -1);
}

void trotRotate::quickTurn(int direction, int degrees) {
  const float degreesPerCycle = 15.0f;
  int cycles = (int)(degrees / degreesPerCycle);
  
  if (cycles < 1) cycles = 1;
  
  rotate(cycles, direction);
}