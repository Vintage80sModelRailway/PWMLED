// PWMOutput.h
#ifndef PWMOutput_h
#define PWMOutput_h

#include <Arduino.h>

class PWMOutput {
  private:

  public: 
    int thrownVal;
    int closedVal;
    int currentPWMVal;
    int requiredPWMVal;
    String currentState;
    String requiredState;
    int stepSize;
    int delayTime;
    int feedbackSensorPin;
    bool lastFeedbackSensorReading;
    bool useSlowMotion;
    bool hasFeedbackSensor;
    bool invertFeedbackSensor;
    bool inDebounce;
    bool isAnLED;
    String jMRIId;
    unsigned long millisAtLastChange;
    unsigned long previousMillis;
    
    PWMOutput();
    PWMOutput(String JMRIId);
    PWMOutput(String JMRIId, int LEDPWMVal);
    PWMOutput(String JMRIId, int ThrownVal, int ClosedVal);
    PWMOutput(String JMRIId,int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFeedbackSensor);   
    PWMOutput(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime);
    PWMOutput(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFeedbackSensor);
    
};

#endif
