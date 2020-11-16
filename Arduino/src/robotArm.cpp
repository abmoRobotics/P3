#include <robotArm.h>
#include <Dynamixel2Arduino.h>

robotArm::robotArm(Dynamixel2Arduino &dxl2){
    dxl = &dxl2;
    const float DXL_PROTOCOL_VERSION = 2.0;
    dxl->begin(57600);
    dxl->setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
    startMotors();
}

double robotArm::getTorque(int motorID)
{
    double measuredTorque = dxl->getPresentCurrent(motorID);
    return measuredTorque;
}

void robotArm::setTorque(int motorID, float goalTorque)
{
    dxl->setGoalPWM(motorID, robotArm::calculatePWM(motorID, goalTorque));
}

int16_t robotArm::getPosition(int motorID)
{
    int16_t measuredPos = dxl->getPresentPosition(motorID);
    float radianPos = ((2 * PI / 4095) * measuredPos);
    char firstByte = (byte)measuredPos;         //
    char secondByte = (byte)(measuredPos >> 8); // Shift 8 bit to left
    Serial.write(firstByte);                    // Write first byte representing a number from 0-255
    Serial.write(secondByte);                   // Write second byte representing a number from 256 til noget stort(ca 32000)
    return measuredPos;
} // Position in radians
void robotArm::setPosition(int motorID, int16_t goalPos)
{
    dxl->setGoalPosition(motorID, goalPos);

}

double robotArm::getVelocity(int motorID)
{
    double measuredVel = dxl->getPresentVelocity(motorID);
    return measuredVel;
}


double robotArm::calculatePWM(int motorid, float torque, int PWM = 0)
{
    float C1MX28{538.42}, C2MX28{149.78}, C1MX64{197.71}, C2MX64{124.29}, C1MX106{120.66}, C2MX106{165.78};
    float angularVel = dxl->getPresentVelocity(motorid) * 0.229 * 0.104719755;

    if(motorid == 1 || motorid == 2)
    {
        PWM = torque * C1MX106 + angularVel * C2MX106;
    }
    else if(motorid == 3)
    {
        PWM = torque * C1MX64 + angularVel * C2MX64;
    }
    else if(motorid == 4 || motorid == 5 || motorid == 6)
    {
        PWM = torque * C1MX28 + angularVel * C2MX28;
    }
    return PWM;
}

void robotArm::setVelocity(int motorID, int goalVel)
{
    dxl->setGoalVelocity(motorID, goalVel);
}


void robotArm::dataGatherer()
{
    
    byte header[5]{};
    byte lenght{};
    Serial3.readBytesUntil(0x00, header, 5);
    if(header[0] == 0xFF && header[1] == 0xFF && header[2] ==  0xFD && header[4] == 0x00)
    {
    int starttime = micros();
    //Serial.println("SUCCESS");
    
    lenght = header[3];
    
    
    
    
    while(Serial3.available() < lenght)
    {
       // Serial.println("WAITING");
    }

    byte ReadData[lenght]{};

        Serial3.readBytes(ReadData, lenght);
        int ID = ReadData[0];
        byte Instruction = ReadData[1];
        


        byte ReadCRC[2]{};
        ReadCRC[0] = ReadData[lenght-2]; 
        ReadCRC[1] = ReadData[lenght-1];

        unsigned int datasize = lenght-4;
        byte Param[datasize]{};
        
        for (size_t i = 0; i < datasize; i++)
        {
            Param[i] = ReadData[2+i];
        }

        // Serial.print("Motor ID: "); Serial.println((int)ID);
        // Serial.print("Length of message: "); Serial.println((int)lenght);
        // Serial.print("Instruction: "); Serial.println((int)Instruction);
        // Serial.print("Data: ");
        // for (size_t i = 0; i < datasize; i++)
        // {
        //     Serial.print(Param[i]); Serial.print("   ");
        // }
        // Serial.println();
        // Serial.print("CRC: "); Serial.print(ReadCRC[0]); Serial.print("   "); Serial.println(ReadCRC[1]);
        int Endtime = micros();
        int Processtime = Endtime-starttime;
        Serial.print("Elapsed time in microseconds: "); Serial.println(Processtime);
    
    

}
}


void robotArm::startMotors(){
for (size_t i = 1; i < 7; i++)
  {
    dxl->torqueOff(i);
    dxl->setOperatingMode(i, OP_PWM);
    
    dxl->torqueOn(i);
  }
}