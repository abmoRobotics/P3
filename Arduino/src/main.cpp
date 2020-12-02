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
  robot = new robotArm(dxl);
//robot->setTorque(2, 0);
//robot->setTorque(4, 0);
//robot->setTorque(3, 0);
//robot->setPosition(2, 2015);
//robot->setPosition(3, 1090);
//robot->setPosition(4, 2045);
}

float secondTime;
float firstTime;
void loop()
{

   if (indexe == 0)
   {
     firstTime = millis();
  }

//if ((indexe % 500) < 250) {  robot->ControlSystem(0, PI/4, -0.1, 0.1);}
//else {robot->ControlSystem(PI/2, 0, 0.1, -0.1);}


robot->ControlSystem(0, 0, 0, 0);

  indexe = indexe + 1;

  // if (robot->dataGatherer()){
  // char command = robot->Instruction;
  // int motorID = robot->MotorID;
  //   if(command == commandList::setJointVelocity)
  //   {
  //     robot->setJointVelocity(motorID, robot->Parameters);//Ændre tallet når vi lige finder ud af det
  //   }
  //   else if (command == commandList::setJointPosition)
  //   {
  //     robot->setJointPositition(motorID, robot->Parameters);//Ændre tallet når vi lige finder ud af det
  //   }
  //   else if (command == commandList::setGripperTorque)
  //   {
  //     robot->setGripperTorque(motorID, robot->Parameters);//Ændre tallet når vi lige finder ud af det
  //   }
  // }
}