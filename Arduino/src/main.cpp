#include <string.h>
#include <Wire.h>
#include "robotArm.h"
#include "LiquidCrystal_I2C.h"

//LiquidCrystal_I2C lcd(0x27, 16, 2);

 Dynamixel2Arduino dxl(DXL_SERIAL, 2);
 robotArm* robot;

void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  robot = new robotArm(dxl);
  //lcd.init();
  //lcd.backlight();
}




void loop()
{
 robot->dataGatherer();
uint8_t command = robot->Instruction;
uint8_t motorID = robot->MotorID;
  //int16_t Value = Data[2] + (Data[3] << 8);
  


 

  if (robot->dataGatherer()){
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
      robot->setVelocity(motorID, robot->Parameters[1]); //Ændre tallet når vi lige finder ud af det
    }

    else if (command == commandList::getTorque)
    {
      robot->getTorque(motorID);
    }
    
    else if (command == commandList::setTorque)
    {
      robot->setTorque(motorID, robot->Parameters[1]);//Ændre tallet når vi lige finder ud af det
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

}