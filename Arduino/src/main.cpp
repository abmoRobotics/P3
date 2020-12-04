#include <string.h>
#include <Wire.h>
#include "robotArm.h"
#include "LiquidCrystal_I2C.h"
#include "soc/rtc_wdt.h"


TaskHandle_t Task1;
TaskHandle_t Task2;
LiquidCrystal_I2C lcd(0x27, 16, 2);


Dynamixel2Arduino dxl(DXL_SERIAL, 4);
robotArm *robot;
int indexe = 0;
void Task1code( void * pvParameters ){
  while(1)
  {
    indexe = indexe + 1;
// if ((indexe % 500) < 250) {  robot->ControlSystem(0, PI/4, -0.1, 0.1);}
// else {robot->ControlSystem(PI/2, 0, 0.1, -0.1);}
robot->ControlSystem(robot->goalPositionJoint1, robot->goalPositionJoint2, robot->goalVelocityJoint3, robot->goalVelocityJoint4);
//robot->ControlSystem(0, 0, 0, 0);
Serial.println(robot->testPos2);

if(robot->openGripper)
{
        robot->openGripperFunc();
}
else if(robot->closeGripper)
{
  for (size_t i = 0; i < 1000; i++)
  {
    robot->closeGripperFunc(robot->gripperTorque);
    robot->ControlSystem(robot->goalPositionJoint1, robot->goalPositionJoint2, robot->goalVelocityJoint3, robot->goalVelocityJoint4);
    delay(1);
  }
  robot->closeGripper = false;
  
}
vTaskDelay(2);
}

}

void Task2code( void * pvParameters ){
  while(1)
  {
if (robot->dataGatherer()){
  char command = robot->Instruction;
  int motorID = robot->MotorID;
    if(command == commandList::setJointVelocity)
    {
      robot->setJointVelocity(motorID, robot->Parameters);//Ændre tallet når vi lige finder ud af det
    }
    else if (command == commandList::setJointPosition)
    {
      robot->setJointPositition(motorID, robot->Parameters);//Ændre tallet når vi lige finder ud af det
    }
    else if (command == commandList::setGripperTorque)
    {
      robot->setGripperTorque(robot->Parameters);//Ændre tallet når vi lige finder ud af det
    }
  }
  // lcd.setCursor(0,0);
  // lcd.print(robot->goalVelocityJoint4);
  // lcd.setCursor(5,0);
  // lcd.print(robot->goalVelocityJoint4);
  // lcd.setCursor(0,1);
  // lcd.print(robot->testPos);
  // lcd.setCursor(5,1);
  // lcd.print((byte)robot->Instruction);
vTaskDelay(1);
}
}


void setup()
{
  Serial.begin(500000);
  robot = new robotArm(dxl);
  pinMode(4, OUTPUT);
  lcd.init();
  lcd.backlight();
//robot->setTorque(2, 0);
//robot->setTorque(4, 0);
//robot->setTorque(3, 0);
//robot->setPosition(2, 2015);
//robot->setPosition(3, 1090);
//robot->setPosition(4, 2045);
rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
rtc_wdt_set_time(RTC_WDT_STAGE0, 25000);
xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    100000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    100000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
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




  

  
}