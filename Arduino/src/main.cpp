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
  //Serial3.begin(115200);
  //pinMode(LED_BUILTIN, OUTPUT);
  robot = new robotArm(dxl);
//robot->setTorque(2, 0);
//robot->setTorque(4, 0);
//robot->setTorque(3, 0);
//robot->setPosition(2, 2015);
//robot->setPosition(3, 1090);
//robot->setPosition(4, 2045);
  //robot->setTorque(4, 0);
}

float secondTime;
float firstTime;
void loop()
{

   if (indexe == 0)
   {
     firstTime = millis();
  }
// if ((indexe % 1000) > 500)
// {
//  robot->ControlSystem(PI, 0, 0, 0);
// }
// else
// {
//  robot->ControlSystem(0, 0, 0, 0);
// }
// if ((indexe % 1000) < 500)
// {
//   int b = indexe%1000;
//  // robot->ControlSystem((PI/2)-0.00314*b, 0, 0, 0);
//   robot->ControlSystem(0, (0)+0.00157*b, 0, 0);
// }
// else {
//  //robot->ControlSystem(0+((indexe%1000)-500)*0.00314, 0, 0, 0);

// robot->ControlSystem(0, (PI/4)-((indexe%1000)-500)* 0.00157, 0, 0);
// }

if ((indexe % 500) < 250)
{
  int b = indexe%1000;
  //robot->ControlSystem((PI/2)-0.00314*b, 0, 0, 0);
  robot->ControlSystem(0, PI/4, -0.1, 0.1);
}
else {
 //robot->ControlSystem(0+((indexe%1000)-500)*0.00314, 0, 0, 0);

robot->ControlSystem(PI/2, 0, 0.1, -0.1);
}

//robot->Tester();

// robot->Tester();

  //int16_t Value = Data[2] + (Data[3] << 8);
  //robot->setTorque2(1, 1.7, 0.2);
  //robot->setTorque(2, -0.0087);
  //robot->setTorque(3, -0.0980);
  //robot->setTorque(4, 0);
  // float positionee;
  // float preTime = millis();

 

  //robot->setTorque2(1,-1.7,-0.3,robot->getPositionRad(1));
 // float a = robot->ControlSystem(0, 0, 0, 0);
//  Serial.println(a);

  // float postTime = millis();

  // float calTime = (postTime - preTime) / 100;
  //Serial.print("Calculated Time: ");
  //Serial.println(calTime);
  //Serial.println(positionee);
// for (size_t i = 0; i < 1000; i++)
// {
//     //  robot->Write_Data(0, 0, 0, 0);
//     //  robot->Tester();
    
// }



//robot->Tester();

  // if (indexe >= 1000)
  // {
  //   if (indexe == 1000)
  //   {
  //     secondTime = millis();
  //   }
  //   Serial.println(secondTime);
  //  float totalTime = secondTime - firstTime;
  //    float sampleTime = totalTime / 1000;
  //    Serial.print("Sample Time: ");
  //    Serial.println(sampleTime);
  // }

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