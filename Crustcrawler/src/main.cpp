#include <string.h>
#include <Wire.h>
#include "robotArm.h"

// #include <LiquidCrystal_I2C.h>


// const int redLED = 7;
// const int yellowLED = 6;
// const int greenLED = 5;

const uint8_t DXL_DIR_PIN = 2;
    




//This namespace is required to use Control table item names





void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(5, OUTPUT);
  // pinMode(6, OUTPUT);
  // pinMode(7, OUTPUT);
  // Use UART port of DYNAMIXEL Shield to debug.
  Serial.begin(115200);

  

//eventuelt lav en funktion til dette
  // for (size_t i = 1; i < 7; i++)
  // {
  //   dxl.torqueOff(i);
  //   dxl.setOperatingMode(i, OP_POSITION);
  //   dxl.torqueOn(i);
  // }



  // lcd.init();                      // initialize the lcd 
  // lcd.backlight();
}


// void blinkLED(int Pin, int time)
// {
//   digitalWrite(Pin, HIGH);
//   delay(time);
//   digitalWrite(Pin, LOW);
//   delay(time);
// }

Dynamixel2Arduino dxl(DXL_SERIAL, 2);
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
  
  
  robotArm robot(dxl);

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