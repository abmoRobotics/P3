#include <Dynamixel.h>

#define DYNAMIXEL_SERIAL Serial2 // change as you want

const uint8_t motor1 = 1;
const uint8_t motor2 = 2;
const uint8_t motor3 = 3;
const uint8_t motor4 = 4;
const uint8_t motor5 = 5;

const uint8_t PIN_RTS = 12;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS);

int dxl_goal_position[2];
void gripperOpen(){
dxl.goalPosition(motor4, dxl.minPositionLimit(motor4));
dxl.goalPosition(motor5, dxl.maxPositionLimit(motor5));
dxl.led(motor4, 1);
dxl.led(motor5, 1);
}

void gripperClose(){
dxl.goalCurrent(motor4, 10000);
dxl.goalCurrent(motor5, 50);
dxl.goalPosition(motor4, dxl.maxPositionLimit(motor4));
dxl.goalPosition(motor5, dxl.minPositionLimit(motor5));
dxl.led(motor4, 0);
dxl.led(motor5, 0);
}

void setup()
{
    Serial.begin(115200);
    
    delay(2000);

    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);
    
    delay(2000);

        dxl.addModel<DxlModel::MX>(1);
        dxl.addModel<DxlModel::MX>(2);
        dxl.addModel<DxlModel::MX>(3);
        dxl.addModel<DxlModel::MX>(4);
        dxl.addModel<DxlModel::MX>(5);

        dxl.torqueEnable(1, true);
        dxl.torqueEnable(2, true);
        dxl.torqueEnable(3, true);
        dxl.torqueEnable(4, true);
        dxl.torqueEnable(5, true);
        
}





void loop()
{
    
    gripperOpen();
    Serial.print(dxl.presentCurrent(motor4));
    Serial.print("                  ");
    Serial.println(dxl.presentCurrent(motor5));
    delay(3000);
    gripperClose();
    delay(3000);
}
