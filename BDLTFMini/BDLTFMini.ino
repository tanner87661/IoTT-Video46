#include <TFMPlus.h>  // Include TFMini Plus Library v1.3.4
#include "printf.h"   // May not work corrrectly with Intel devices
#include <LocoNet.h>
#include <Adafruit_PWMServoDriver.h>
#include <Math.h>
#include <SoftwareSerial.h>       // alternative software serial library

#define polSwi 15 //switch address of switch used to change polarity
#define hatRxD 2
#define hatTxD 3
#define lnTx 7
#define relPin1 11
#define relPin2 12

#define servoMin 120 //technical minimum position for Servo
#define servoMax 620 //technical maximum position for Servo
#define rotSpeed  500 //increments per second, this is the servo rotation speed to determin wait time between setting servo and taking measurement

#define distIntv 150    //minimum time between two distance sensor readings
#define servoChannel 0  //servo channel used for this servo

uint32_t distReadTime = millis() + distIntv; //the next available time for reading the distance sensor

uint16_t tfDist;       // Distance measurement in centimeters (default)
uint16_t tfFlux;       // Luminous flux or intensity of return signal
uint16_t tfTemp;       // Temperature in degrees Centigrade (coded)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
TFMPlus tfmP;         // Create a TFMini Plus object
SoftwareSerial mySerial(hatRxD, hatTxD, false);  // Choose the correct RX, TX Pins
lnMsg        *LnPacket;

uint16_t sensorIndex = 0; //current sensor to be read
uint32_t awaitServoPosTime; //expected time when servo is in position
uint16_t servoPos = servoMax; //last known position of servo

typedef struct
{
  uint16_t sensorPWM; //must be between servoMin and servoMax
  uint16_t distMin; //distance where valid sensor range startes in cm
  uint16_t distMax; //distance where valid sensor range ends in cm
  uint16_t currDist; //last reading data
  uint16_t currFlux; //last reading data
  uint16_t currTemp; //last reading data
  bool     newData; //data has been overwritten since last pocessing
  bool     lastLevel; //value of last sensor reading to determine if there was a change that needs to be sent to LocoNet
  uint16_t bdNr; //block detector number
} sensorPos;

#define numDetPos 2
sensorPos sensorList[numDetPos] = {
//  {130, 45, 65, 0, 0, 0, false, 0, 1},
  {390, 60, 90, 0, 0, 0, false, 0, 4},
  {345, 55, 90, 0, 0, 0, false, 0, 5},
//  {590, 25, 45, 0, 0, 0, false, 0, 4},
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  printf_begin();          // Initialize printf.
  LocoNet.init(lnTx);
  pwm.begin(); //Adafruit library initialization
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  
  awaitServoPosTime = commandServoPos(servoChannel, servoMin); //bring servo to defined minimum position
  delay(1000); //make sure it is there

  initMiniPlus();

  pinMode(relPin1, OUTPUT); //set the relay pins
  pinMode(relPin2, OUTPUT);

  awaitServoPosTime = commandServoPos(servoChannel, sensorList[sensorIndex].sensorPWM); //get reading process started
}

bool receiveLocoNet()
{
  LnPacket = LocoNet.receive() ;
  if ( LnPacket ) 
    if (!LocoNet.processSwitchSensorMessage(LnPacket)) 
      Serial.println();
  return LnPacket;
}

uint32_t commandServoPos(uint8_t servoNr, uint16_t newPos)
{
  int moveDistance = abs((int)newPos - (int)servoPos);
  int moveTime = round(1000 * ((float)moveDistance / (float)rotSpeed)); //ms
  pwm.setPWM(servoNr, 0, newPos);
  servoPos = newPos;
//  printf("Incr: %i Wait: %i\n", moveDistance, moveTime);
  return millis() + moveTime;
}

void processServoReading()
{
  if (millis() > awaitServoPosTime) //servo in position, so read distance, then set next position
  {
    if (millis() > distReadTime) //sensor ready for new readout
    {
      if (tfmP.getData(tfDist, tfFlux, tfTemp)) // Get data from the device to clear the buffer and ensure fresh reading
        if (tfmP.getData(tfDist, tfFlux, tfTemp)) // Get data from the device
          if (tfFlux > 0)
          {
            sensorList[sensorIndex].currDist = tfDist;      
            sensorList[sensorIndex].currFlux = tfFlux;      
            sensorList[sensorIndex].currTemp = tfTemp;      
            sensorList[sensorIndex].newData = true;    
//            printf("Pos: %i Angle: %i Dist: %i Flux: %i\n", sensorIndex, sensorList[sensorIndex].sensorPWM, tfDist, tfFlux);
            sensorIndex = (sensorIndex + 1) % numDetPos;
            awaitServoPosTime = commandServoPos(servoChannel, sensorList[sensorIndex].sensorPWM); //get reading process started
            distReadTime = millis() + distIntv;
          }
          //else do nothing, try again in next cycle
    }
    //else do nothing, try again in next cycle
  }
  //else do nothing, come back and check again
}

void processInputData()
{
  for (int i=0; i < numDetPos; i++)
  {
    if (sensorList[i].newData)
    {
      bool isActive = ((sensorList[i].currDist >= sensorList[i].distMin) && (sensorList[i].currDist <= sensorList[i].distMax));
      if (isActive != sensorList[i].lastLevel)
      {
        sensorList[i].lastLevel = isActive;
        //send LocoNet Message
        sendSensor(sensorList[i].bdNr, sensorList[i].lastLevel);
        //do local processing
        localProcessing(i);
      }
      sensorList[i].newData = false;
    }
  }
}

void localProcessing(int sensorIndex)
{
  if (sensorList[sensorIndex].lastLevel)
    switch (sensorIndex)
    {
      case 0: 
        sendSwitch(polSwi, 1, 1); break;
      case 1: 
        sendSwitch(polSwi, 0, 1); break;
    }
}

void loop() 
{
  // put your main code here, to run repeatedly:
  while (receiveLocoNet());
  processServoReading();
  while (receiveLocoNet());
  processInputData();
}
