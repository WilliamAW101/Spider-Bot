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

void trotForward::Trot(int cycles, int direction) {
  const int frames = 80;
  const float stanceRatio = 0.3f;
  const float tibiaGrip = 15.0f;  // Adjust this value - degrees to pull foot inward during stance
  
  for (int cycle = 0; cycle < cycles; cycle++) {
    
    // FL + RR swing
    for (int f = 0; f < frames; f++) {
      float t = (float)f / (frames - 1);
      float ease = easeInOutQuad(t);
      
      // FL goes back and RR goes forward
      float swingProgress = -0.5f + ease;  
      float swingX_FL = -direction * swingProgress * FL->stepLength;  // Left leg
      float swingX_RR = direction * swingProgress * RR->stepLength;   // Right leg (opposite)
      
      // Swing height and tibia position
      float swingZ, tibiaZ_swing;
      if (t < stanceRatio) {
        swingZ = 0.0f;
        tibiaZ_swing = tibiaGrip;  // Foot pulled in during touchdown
      } else {
        swingZ = FL->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
        tibiaZ_swing = 0.0f;  // Foot neutral during swing
      }
      
      FL->leg->moveToPosition(swingX_FL, swingZ, tibiaZ_swing);
      RR->leg->moveToPosition(swingX_RR, -swingZ, tibiaZ_swing);
      
      // Stance legs: FR (right) and RL (left) push with foot pulled inward
      float stanceProgress = 0.5f - ease;  // +0.5 to -0.5 (opposite of swing)
      float stanceX_FR = direction * stanceProgress * FR->stepLength;   // Right leg
      float stanceX_RL = -direction * stanceProgress * RL->stepLength;  // Left leg
      
      // Gradually release tibia grip as leg pushes back
      float tibiaZ_stance = tibiaGrip * (1.0f - t);  // Full grip at start, releases by end
      
      FR->leg->moveToPosition(stanceX_FR, 0, tibiaZ_stance);
      RL->leg->moveToPosition(stanceX_RL, 0, tibiaZ_stance);
      
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
      
      float swingProgress = -0.5f + ease;
      float swingX_FR = direction * swingProgress * FR->stepLength;   // Right leg
      float swingX_RL = -direction * swingProgress * RL->stepLength;  // Left leg
      
      // Swing height and tibia position
      float swingZ, tibiaZ_swing;
      if (t < stanceRatio) {
        swingZ = 0.0f;
        tibiaZ_swing = tibiaGrip;  // Foot pulled in during touchdown
      } else {
        swingZ = FR->stepHeight * sinf((t - stanceRatio) / (1.0f - stanceRatio) * M_PI);
        tibiaZ_swing = 0.0f;  // Foot neutral during swing
      }
      
      FR->leg->moveToPosition(swingX_FR, swingZ, tibiaZ_swing);
      RL->leg->moveToPosition(swingX_RL, -swingZ, tibiaZ_swing);
      
      // Stance legs with tibia grip
      float stanceProgress = 0.5f - ease;
      float stanceX_FL = -direction * stanceProgress * FL->stepLength;  // Left leg
      float stanceX_RR = direction * stanceProgress * RR->stepLength;   // Right leg
      
      float tibiaZ_stance = tibiaGrip * (1.0f - t);  // Gradually release grip
      
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