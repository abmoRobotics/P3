#include <string.h>
#include <Wire.h>
#include "robotArm.h"
#include "LiquidCrystal_I2C.h"

//LiquidCrystal_I2C lcd(0x27, 16, 2);

Dynamixel2Arduino dxl(DXL_SERIAL, 2);
robotArm *robot;
int indexe = 0;
void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  robot = new robotArm(dxl);
  //robot->setTorque(2, 0);
 // robot->setTorque(1, 0);
  robot->setTorque(3, 0);

  robot->setTorque(4, 0);
  //lcd.init();
  //lcd.backlight();
}

float secondTime;
float firstTime;
void loop()
{
  if (indexe == 0)
  {
    firstTime = millis();
  }

  //int16_t Value = Data[2] + (Data[3] << 8);
  //robot->setTorque2(1, 1.7, 0.2);
  //robot->setTorque(2, -0.0087);
  //robot->setTorque(3, -0.0980);
  //robot->setTorque(4, 0);
  float positionee;
  float preTime = millis();

 // robot->setPosition(1, 1290);
  robot->setPosition(3, 1090);
  robot->setPosition(4, 2045);
  //robot->setTorque2(1,-1.7,-0.3,robot->getPositionRad(1));
  float a = robot->ControlSystem(0, 0, 0, 0);
  Serial.println(a);

  float postTime = millis();

  float calTime = (postTime - preTime) / 100;
  //Serial.print("Calculated Time: ");
  //Serial.println(calTime);
  //Serial.println(positionee);

  // if (indexe >= 1000)
  // {
  //   if (indexe == 1000)
  //   {
  //     secondTime = millis();
  //   }
  //   Serial.println(secondTime);
  //   float totalTime = secondTime - firstTime;
  //   float sampleTime = totalTime / 1000;
  //   Serial.print("Sample Time: ");
  //   Serial.println(sampleTime);
  // }

  indexe = indexe + 1;

  /*
  if (robot->dataGatherer()){
  char command = robot->Instruction;
  int motorID = robot->MotorID;
    if(command == commandList::setPosition)
    {
      robot->setPosition(motorID, robot->Parameters[1]);//Ændre tallet når vi lige finder ud af det
    }

    else if(command == commandList::getPosition) 
    {
    int16_t pos = robot->getPosition(motorID);
  
    }

    else if(command == commandList::getVelocity) 
    {
      robot->getVelocity(motorID);
    }

    else if(command == commandList::setVelocity)
    {
      robot->setVelocity(motorID, robot->Parameters); //Ændre tallet når vi lige finder ud af det
    }

    else if (command == commandList::getTorque)
    {
      robot->getTorque(motorID);
    }
    
    else if (command == commandList::setTorque)
    {
      robot->setTorque(motorID, robot->Parameters);//Ændre tallet når vi lige finder ud af det
    }
    else if (command == commandList::setGripperTorque)
    {
      robot->setGripperTorque(motorID, robot->Parameters);//Ændre tallet når vi lige finder ud af det
    }
    
  }
 
  // if (useData == true)
  // {
  //   if (command == 10)
  //   {
  //     communication::setTorque(motorID, Value, dxl);
  //   }

  // else if(command == commandList::getTorque)
  // {
  //     communication::getTorque(motorID, dxl);
  // }
    
  // else if(command == commandList::getPosition)
  // {    
    
  //   	communication::getPosition(motorID,dxl);

      
  //    }
  //   else if(command == commandList::setPosition)
  //   {
  //     communication::setPosition(motorID, Value, dxl);
  // }

  //   else if(command == commandList::getVelocity) 
  //   {
  //   communication::getVelocity(motorID, dxl);
  //   }

  //   else if(command == commandList::setVelocity)
  //   {
  //   communication::setVelocity(motorID, Value, dxl);
  //   }f
   
  //   else
  //   {
    
  //     digitalWrite(13, HIGH);
  //     delay(500);
  //     digitalWrite(13, LOW);
  //     delay(500);
  //   }
  //   useData = false;
  // }
*/
}