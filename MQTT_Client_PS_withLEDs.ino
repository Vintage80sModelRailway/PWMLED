#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "PWMBoard.h"
#include "PWMOutput.h"

#define NumberOfPWMBoards 1
#define OutputTurnoutThresholdID 2041

// Update these with values suitable for your network.
byte mac[6] = { 0x90, 0xA2, 0xDA, 0x58, 0x66, 0x25 };
IPAddress ip(192, 168, 1, 132);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress server(192, 168, 1, 29);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
EthernetClient ethClient;
PubSubClient client(ethClient);

PWMBoard PWMBoards[NumberOfPWMBoards];

bool inSetup = true;

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println("New message " + String(topic) + " - " + message);
  ProcessIncomingMessage(message, String(topic));
}

void setup()
{
  Serial.begin(19200);

  client.setServer(server, 1883);
  client.setCallback(callback);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm.begin();
  //pwm.setOscillatorFrequency(27000000);
 
  Ethernet.init(53);
  //Ethernet.begin(mac, ip, gateway, subnet);
  Ethernet.begin(mac);

  // Allow the hardware to sort itself out
  delay(1500);

  if (!client.connected()) {
    reconnect();
  }
  Serial.println("Finished connection");
  InitialiseConfig();
  inSetup = false;
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  
  for (int board = 0; board < NumberOfPWMBoards; board++) {
    for (int pin = 0; pin < PWMBoards[board].numberOfOutputs; pin++) {
      if (PWMBoards[board].outputs[pin].useSlowMotion) {
        ProcessPointsMoveWithSpeedControl(board, pin);
      }
    }
  }
  client.loop();
}

void ProcessIncomingMessage(String message, String topic) {
  String strTopic = (String)topic;
  int pos = strTopic.lastIndexOf("/");
  int pin = -1;
  int boardId = -1;
  if (pos >= 0 && strTopic.indexOf("turnout") >= 0) {
    int outputstatus = 0;
    String justTheID = strTopic.substring(pos + 1);
    int iID = justTheID.toInt();

    //Find turnout with this ID
    for (int board = 0; board < NumberOfPWMBoards; board++) {
      for (int pin = 0; pin < PWMBoards[board].numberOfOutputs; pin++) {
        if (PWMBoards[board].outputs[pin].jMRIId == justTheID) {
          PWMBoards[board].outputs[pin].requiredState = message;
          Serial.println(justTheID + " - needs to go " + message);
          if (PWMBoards[board].outputs[pin].isAnLED) {
            int requiredPWM = 0;
            if (message == "ON") {
              requiredPWM = PWMBoards[board].outputs[pin].thrownVal;
            }
            PWMBoards[board].pwm.setPWM(pin, 0, requiredPWM);
          }
          else if (PWMBoards[board].outputs[pin].useSlowMotion) {
            PWMBoards[board].outputs[pin].requiredState = message;
            if (message == "CLOSED") {
              PWMBoards[board].outputs[pin].requiredPWMVal = PWMBoards[board].outputs[pin].closedVal;
              if (inSetup) {
                PWMBoards[board].outputs[pin].currentPWMVal = PWMBoards[board].outputs[pin].closedVal + 1;
                Serial.println("Setup closed "+justTheID+" "+String(PWMBoards[board].outputs[pin].currentPWMVal));
              }
            } else {
              PWMBoards[board].outputs[pin].requiredPWMVal = PWMBoards[board].outputs[pin].thrownVal;
              if (inSetup) {
                PWMBoards[board].outputs[pin].currentPWMVal = PWMBoards[board].outputs[pin].thrownVal - 1;
                Serial.println("Setup thrown "+justTheID+" "+String(PWMBoards[board].outputs[pin].currentPWMVal));
              }
              Serial.println("Required PWM VAL " + String(PWMBoards[board].outputs[pin].requiredPWMVal));
              Serial.println("Current PWM VAL " + String(PWMBoards[board].outputs[pin].currentPWMVal));
            }
          }
          else {
            MoveServoFast(board, pin, PWMBoards[board].outputs[pin], message);
          }
          break;
        }
      }
    }
  }
}

void PublishToMQTT(String topic, String message)  {
  byte topicBuffer[topic.length() + 1];
  byte messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  client.publish(topicBuffer, messageBuffer, message.length(), true);
}

void MoveServoFast(int board, int pin, PWMOutput thisTurnout, String message) {
  Serial.println("Closed " + String(thisTurnout.closedVal) + " Thrown " + String(thisTurnout.thrownVal));
  bool servoMoved = false;
  if (message == "CLOSED") {
    if (thisTurnout.closedVal > 800 && thisTurnout.closedVal < 2200) {
      PWMBoards[board].pwm.writeMicroseconds(pin, thisTurnout.closedVal);
      servoMoved = true;
    }
  }
  else if (message == "THROWN" && thisTurnout.thrownVal > 800 && thisTurnout.thrownVal < 2200) {
    PWMBoards[board].pwm.writeMicroseconds(pin, thisTurnout.thrownVal);
    servoMoved = true;
  }

  if (servoMoved) PWMBoards[board].outputs[pin - PWMBoards[board].CMRIIndexModifier].currentState = message;
}

void HandlePWMLEDSwitch(int board, int pin, int PWMVal) {
  
}

bool ProcessPointsMoveWithSpeedControl(int board, int pin)
{
  //Serial.println("Number of servos moving "+String(numberOfServosMoving));
  bool moveIsComplete = false;
  unsigned long currentMillis = millis();
  int requiredPosition = PWMBoards[board].outputs[pin].requiredPWMVal;

  //    int totaldStepSize =  PWMBoards[board].outputs[pin].stepSize + calculatedStepSize;
  int totaldStepSize =  PWMBoards[board].outputs[pin].stepSize;
  if ((requiredPosition != PWMBoards[board].outputs[pin].currentPWMVal && currentMillis - PWMBoards[board].outputs[pin].previousMillis >= PWMBoards[board].outputs[pin].delayTime))
  {
    //Serial.println(String(PWMBoards[board].outputs[pin].currentPWMVal && currentMillis - PWMBoards[board].outputs[pin].previousMillis)+" delay time "+String(PWMBoards[board].outputs[pin].delayTime));
    PWMBoards[board].outputs[pin].previousMillis = currentMillis;

    //Protect thee motors from accidentally silly, potentially damaging values
    if (requiredPosition > 800 && requiredPosition < 2351)
    {
      if (requiredPosition > PWMBoards[board].outputs[pin].currentPWMVal)
      {
        int intendedPWMValue = PWMBoards[board].outputs[pin].currentPWMVal + totaldStepSize;
        if (intendedPWMValue > requiredPosition)
        {
          intendedPWMValue = requiredPosition;
        }

        PWMBoards[board].outputs[pin].currentPWMVal = intendedPWMValue;
        //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(intendedPWMValue) + ".");
        PWMBoards[board].pwm.writeMicroseconds(pin, intendedPWMValue);

        //if required position is now equal to current position, set frog polarity too
        if (requiredPosition == intendedPWMValue)
        {
          moveIsComplete = true;
        }
      }
      else// (requiredPosition < CurrentPWMValue[pin])
      {
        int intendedPWMValue = PWMBoards[board].outputs[pin].currentPWMVal - totaldStepSize;
        if (intendedPWMValue < requiredPosition)
        {
          intendedPWMValue = requiredPosition;
        }

        PWMBoards[board].outputs[pin].currentPWMVal = intendedPWMValue;
        //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(intendedPWMValue) + ".");
        PWMBoards[board].pwm.writeMicroseconds(pin, intendedPWMValue);

        //if required position is now equal to current position, set frog polarity too
        if (requiredPosition == intendedPWMValue)
        {
          moveIsComplete = true;
        }
      }
    }
    else {
      //Serial.println(String(PWMBoards[board].outputs[pin].currentPWMVal && currentMillis - PWMBoards[board].outputs[pin].previousMillis)+" delay out of  time "+String(PWMBoards[board].outputs[pin].delayTime));
    }

    //Set frog polarity by bit value not servo position as some outputs are inverted
    if (moveIsComplete) {
      PWMBoards[board].outputs[pin].currentState = PWMBoards[board].outputs[pin].requiredState;
      PWMBoards[board].outputs[pin].currentPWMVal = PWMBoards[board].outputs[pin].requiredPWMVal;
    }
  }
  return moveIsComplete;
}

void reconnect() {
  // Loop until we're reconnected

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClientNode2")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic","hello world");
      // ... and resubscribe
      //client.subscribe("#");
      client.subscribe("track/turnout/2001");
      client.subscribe("track/turnout/2002");
      client.subscribe("track/turnout/2003");
      client.subscribe("track/turnout/2004");
      client.subscribe("track/turnout/2005");
      client.subscribe("track/turnout/2006");
      client.subscribe("track/turnout/2007");
      client.subscribe("track/turnout/2008");

      client.subscribe("track/turnout/2041");
      client.subscribe("track/turnout/2042");
      client.subscribe("track/turnout/2043");
      client.subscribe("track/turnout/2044");
      client.subscribe("track/turnout/2045");
      client.subscribe("track/turnout/2046");

      client.subscribe("track/turnout/2049");
      client.subscribe("track/turnout/2050");
      client.subscribe("track/turnout/2051");
      client.subscribe("track/turnout/2052");
      client.subscribe("track/turnout/2053");
      client.subscribe("track/turnout/2054");

      client.subscribe("track/light/LEDJMRIID1");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void InitialiseConfig() {
  //Board 1 - under upper incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  PWMBoards[0].numberOfOutputs = 8;
  PWMBoards[0].outputs[0] = PWMOutput("2001", 1000, 1800, 1, 5, 3, false); //thrown, closed,feedback sensor pin, invertfrog, invert sensor
  PWMBoards[0].outputs[1] = PWMOutput("2002", 1650, 1150, 1, 5, 4, true); //thrown, closed, invertfrog
  PWMBoards[0].outputs[2] = PWMOutput("2003", 1200, 2100, 1, 5, 7, true); //thrown, closed, invertfrog
  PWMBoards[0].outputs[3] = PWMOutput("2004", 1290, 1800, 1, 5, 10, false); //thrown, closed, invertfrog
  PWMBoards[0].outputs[4] = PWMOutput("2005", 1700, 1250, 1, 5, 8, true); //thrown, closed, invertfrog
  PWMBoards[0].outputs[5] = PWMOutput("2006", 1810, 1200, 1, 5, 9, false); //thrown, closed, invertfrog
  PWMBoards[0].outputs[6] = PWMOutput("2007", 1850, 1200, 1, 5, 6, true); //thrown, closed, invertfrog
  PWMBoards[0].outputs[7] = PWMOutput("2008", 2000, 1250, 1, 5, 5, false); //thrown, closed, invertfrog

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

  PWMBoards[1].pwm = Adafruit_PWMServoDriver(0x41);
  PWMBoards[1].numberOfOutputs = 8;
  PWMBoards[1].outputs[0] = PWMOutput("LEDJMRIID1", 3096);


  PWMBoards[1].pwm.begin();
  PWMBoards[1].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[1].pwm.setOscillatorFrequency(25000000);

  Serial.println("Setup complete");
}
