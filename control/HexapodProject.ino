#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "spiderLeg.h"
#include "Gait.h"
#include "trotForward.h"
#include "trotRotate.h"
#include "arm.h"
#include <SoftwareSerial.h>

SoftwareSerial piSerial(10, 11); // RX=10 TX=11

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

SpiderLeg leg1(&pwm, 0, 1, 2, 120, 130, 100);    // Front Left
SpiderLeg leg2(&pwm, 3, 4, 5, 140, 120, 100);     // Front Right
SpiderLeg leg3(&pwm, 6, 7, 8, 120, 130, 100);     // Rear Left
SpiderLeg leg4(&pwm, 9, 10, 11, 100, 110, 100);   // Rear Right

float stepLength = 50;
float stepHeight = 40;
int stepDelay = 5;
int restDelay = 50;
int xValue = 0;
float stepLengthRotate = 30;
float stepHeightRotate = 50;
int stepDelayRotate = 5;

Gait gait1(&leg1, stepLength, stepHeight, stepDelay);
Gait gait2(&leg2, stepLength, stepHeight, stepDelay);
Gait gait3(&leg3, stepLength, stepHeight, stepDelay);
Gait gait4(&leg4, stepLength, stepHeight, stepDelay);
trotForward spiderTrot(&gait1, &gait2, &gait3, &gait4);

Gait gait1R(&leg1, stepLengthRotate, stepHeightRotate, stepDelayRotate);
Gait gait2R(&leg2, stepLengthRotate, stepHeightRotate, stepDelayRotate);
Gait gait3R(&leg3, stepLengthRotate, stepHeightRotate, stepDelayRotate);
Gait gait4R(&leg4, stepLengthRotate, stepHeightRotate, stepDelayRotate);
trotRotate spiderRotate(&gait1R, &gait2R, &gait3R, &gait4R, restDelay);

// hatch arm
arm penniArm(&pwm, 12);

void setup() {
  Serial.begin(9600);
  piSerial.begin(9600);
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
  
  penniArm.init();
  penniArm.open();
  
  delay(1000); // get everything setup
  Serial.println("Ready to walk!");
}

static char currentCommand = 'S'; 

void loop() {

  // Heartbeat
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.println("Waiting for Pi data...");
    lastPrint = millis();
  }

  // Pi Control
  
  // Check for new command from Pi
  if (piSerial.available()) {
    char incomingChar = piSerial.read();
    Serial.println(incomingChar);
    // Only update if command actually changed
    if (incomingChar != currentCommand) {
      currentCommand = incomingChar;
      Serial.print("Command changed to: ");
      Serial.println(currentCommand);
    }
  }

  switch (currentCommand) {
    case 'F':
      spiderTrot.Trot(1, -1);
      break;
    
    case 'B':
      spiderTrot.Trot(1, 1);
      break;
      
    case 'L':
      spiderRotate.rotateCounterClockwise(1);
      break;
      
    case 'R':
      spiderRotate.rotateClockwise(1);
      break;
      
    case 'N':
      spiderTrot.prepareStance();
      break;
    
    case 'O':
      penniArm.open();
      break;
    
    case 'C':
      penniArm.close();
      break;
      
    default:
      Serial.println("Unknown command");
      spiderTrot.prepareStance(); // just stop if the command is wack
      currentCommand = 'N';  
  }

  // manual control
  
  // xValue = analogRead(A0);
  // Serial.print("x = ");
  // Serial.println(xValue);

  // if (xValue > 700) {
  //   spiderTrot.Trot(1, -1);
  // } else if (xValue < 200) {
  //   spiderRotate.rotateClockwise(1);
  // } 
  // else {  // stopping
  //   spiderTrot.prepareStance();
  //   penniArm.open();
  //   delay(1000);
  //   penniArm.close();
  //   delay(1000);
  // }
  // spiderTrot.prepareStance();

  // arm testing stuff

  // penniArm.open();
  // delay(1000);
  // penniArm.close();
  // delay(1000);
}