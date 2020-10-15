#include <string.h>
#include <Wire.h>
#include "robotArm.h"

const uint8_t DXL_DIR_PIN = 2;
    
void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
 
  Serial.begin(115200);
}


Dynamixel2Arduino dxl(DXL_SERIAL, 2);
robotArm robot(dxl);

void loop()
{

  bool useData = false;
  int Data[256];    //OBS: ændret fra int; First byte = command, second byte = motor id, third and fourth byte = value(int16_t)
  int i = 0;
  while (Serial.available() > 0)
  {
    Data[i] = Serial.read();

    i++;
    useData = true;
  }

  uint8_t command = Data[0];
  uint8_t motorID = Data[1];
  uint16_t Value = Data[2] + (Data[3] << 8);
  
  
  

  if (useData == true){
    if(command == commandList::setPosition)
    {
      robot.setPosition(motorID, Value);
  }

    else if(command == commandList::getPosition) 
    {
    robot.getPosition(motorID);
    }

    delay(1000);
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
  delay(1000);
}