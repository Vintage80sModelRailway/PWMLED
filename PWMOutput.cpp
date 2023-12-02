// PWMOutput.cpp
#include "PWMOutput.h"

PWMOutput::PWMOutput() {
  useSlowMotion = false;
  hasFeedbackSensor = false;
  currentPWMVal = 1500;
  isAnLED = false;
}

PWMOutput::PWMOutput(String JMRIId, int ThrownVal, int ClosedVal) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  currentPWMVal = 1500;
  isAnLED = false;
}

PWMOutput::PWMOutput(String JMRIId, int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFeedbackSensor) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  useSlowMotion = false;
  inDebounce = false;
  millisAtLastChange = 5000;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
  isAnLED = false;
}

PWMOutput::PWMOutput(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  hasFeedbackSensor = false;
  currentPWMVal = 1500;
  isAnLED = false;
}

PWMOutput::PWMOutput(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFeedbackSensor) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  millisAtLastChange = 5000;
  inDebounce = false;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
  isAnLED = false;
}

PWMOutput::PWMOutput(String JMRIId) {
  jMRIId = JMRIId;
  isAnLED = false;
}

PWMOutput::PWMOutput(String JMRIId, int LEDPWMVal) {
  isAnLED = true;
  thrownVal = LEDPWMVal;
  closedVal = 0;
}
