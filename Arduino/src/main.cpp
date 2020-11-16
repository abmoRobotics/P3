#include <string.h>
#include <Wire.h>
#include "robotArm.h"
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

 Dynamixel2Arduino dxl(DXL_SERIAL, 2);
 robotArm* robot;

void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  robot = new robotArm(dxl);
  lcd.init();
  lcd.backlight();
}




void loop()
{
  robot->dataGatherer();
  bool useData = false;
  int Data[256];    //OBS: ændret fra int; First byte = command, second byte = motor id, third and fourth byte = value(int16_t)
  int i = 0;
  
  // while (Serial.available() > 0)
  // {
  //   Data[i] = Serial.read();
  //   i++;
  //   useData = true;
  // }

  uint8_t command = Data[0];
  uint8_t motorID = Data[1];
  int16_t Value = Data[2] + (Data[3] << 8);
  


 

  if (useData == true){
    if(command == commandList::setPosition)
    {
      robot->setPosition(motorID, Value);
      lcd.clear();
      lcd.setCursor(0,1);
  lcd.print(command);
  lcd.setCursor(5,1);
  lcd.print(motorID);
  lcd.setCursor(10,1);
  lcd.print(Value);
  }

    else if(command == commandList::getPosition) 
    {
    int16_t pos = robot->getPosition(motorID);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(pos);
      lcd.setCursor(0,1);
      lcd.print(command);
      lcd.setCursor(5,1);
     lcd.print(motorID);
  
    }

    else if(command == commandList::getVelocity) 
    {
      robot->getVelocity(motorID);
    }

    else if(command == commandList::setVelocity)
    {
      robot->setVelocity(motorID, Value); //i tvivl om vi skaljghg bruge Value som variable. 
      //men vi sender vel kun en værdi, enten position, torque eller velocity, right?
    }

    else if (command == commandList::getTorque)
    {
      robot->getTorque(motorID);
    }
    
    else if (command == commandList::setTorque)
    {
      robot->setTorque(motorID, Value);
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