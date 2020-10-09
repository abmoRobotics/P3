#include <string.h>
#include "communication.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);



const uint8_t Elbow = 1;
const uint8_t Forearm = 2;
const uint8_t Hand_vert = 3;
const uint8_t Hand_hori = 4;
const uint8_t GripperR = 5;
const uint8_t GripperL = 6;



const int redLED = 7;
const int yellowLED = 6;
const int greenLED = 5;


const float DXL_PROTOCOL_VERSION = 2.0;



//This namespace is required to use Control table item names
using namespace ControlTableItem;


#define DXL_SERIAL Serial1
const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  // Use UART port of DYNAMIXEL Shield to debug.
  Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (size_t i = 1; i < 7; i++)
  {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_POSITION);
    dxl.torqueOn(i);
  }



  lcd.init();                      // initialize the lcd 
  lcd.backlight();
}


void blinkLED(int Pin, int time)
{
  digitalWrite(Pin, HIGH);
  delay(time);
  digitalWrite(Pin, LOW);
  delay(time);
}


void loop()
{

  bool useData = false;
  int Data[3];
  int i = 0;
  while (Serial.available() > 0)
  {
    Data[i] = Serial.read();
    i++;
    useData = true;
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(Data[0]);
  lcd.setCursor(5,0);
  lcd.print(Data[1]);
  lcd.setCursor(9,0);
  lcd.print(Data[2]);

  if (useData == true)
  {
    switch (Data[0])
    {
    case 10:
      communication::setTorque(Data[1], Data[2], dxl);
    break;

    case 11:
      communication::getTorque(Data[1], dxl);
      break;
    
    case 12:
      int test = communication::getPosition(Data[1], dxl);
      Serial.write((int)communication::getPosition(Data[1], dxl));
      lcd.setCursor(0,1);
      lcd.print(test);
    break;

    case 13:
      communication::setPosition(Data[1], Data[2], dxl);
    break;

    case 14: 
    communication::getVelocity(Data[1], dxl);
    break;

    case 15:
    communication::setVelocity(Data[1], Data[2], dxl);
    break;
    
    default:
      digitalWrite(13, HIGH);
      delay(500);
      digitalWrite(13, LOW);
      delay(500);
      break;
    }
    useData = false;
  }
  delay(1000);
}