// PWMBoard.h
#ifndef PWMBoard_h
#define PWMBoard_h

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "PWMOutput.h"

class PWMBoard {
  private:
  public: 
    int numberOfOutputs;
    int CMRIIndexModifier;
    //List<Turnout> turnouts;
    PWMOutput outputs[8];
    Adafruit_PWMServoDriver pwm;  
};

#endif
