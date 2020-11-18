#include <robotArm.h>
#include <Dynamixel2Arduino.h>

robotArm::robotArm(Dynamixel2Arduino &dxl2)
{
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

double robotArm::calculatePWM(int motorid, float torque)
{
    int PWM;
    float C1MX28{538.42}, C2MX28{149.78}, C1MX64{197.71}, C2MX64{124.29}, C1MX106{120.66}, C2MX106{165.78};
    float angularVel = dxl->getPresentVelocity(motorid) * 0.229 * 0.104719755;

    if (motorid == 1 || motorid == 2)
    {
        PWM = torque * C1MX106 + angularVel * C2MX106;
    }
    else if (motorid == 3)
    {
        PWM = torque * C1MX64 + angularVel * C2MX64;
    }
    else if (motorid == 4 || motorid == 5 || motorid == 6)
    {
        PWM = torque * C1MX28 + angularVel * C2MX28;
    }
    return PWM;
}

void robotArm::setVelocity(int motorID, int goalVel)
{
    dxl->setGoalVelocity(motorID, goalVel);
}

bool robotArm::dataGatherer()
{

    bool debug = true;
    byte header[5]{};
    byte lenght{};
    Serial.readBytesUntil(0x00, header, 5);
    if (header[0] == 0xFF && header[1] == 0xFF && header[2] == 0xFD && header[4] == 0x00)
    {
        int starttime = micros();

        lenght = header[3];

        while (Serial.available() < lenght)
        {
            // Serial.println("WAITING");
        }

        byte ReadData[lenght]{};

        Serial.readBytes(ReadData, lenght);
        int ID = ReadData[0];
        byte Instruction = ReadData[1];

        byte ReadCRC[2]{};
        ReadCRC[0] = ReadData[lenght - 2];
        ReadCRC[1] = ReadData[lenght - 1];
        unsigned short RecievedCRC = (ReadCRC[0] << 8) | ReadCRC[1];

        unsigned int datasize = lenght - 4;
        byte Param[datasize]{};

        for (size_t i = 0; i < datasize; i++)
        {
            Param[i] = ReadData[2 + i];
        }

        byte CRCArray[5 + lenght - 2];

        for (size_t i = 0; i < sizeof(header); i++)
        {
            CRCArray[i] = header[i];
        }
        for (size_t i = 0; i < sizeof(ReadData) - 2; i++)
        {
            CRCArray[i + 5] = ReadData[i];
        }

        unsigned short CalcCRC = robotArm::CalculateCRC(0, CRCArray, sizeof(CRCArray));
        
        if(debug == true)
        {
        int Endtime = micros();
        int Processtime = Endtime - starttime;

        //Serial3.write(ReadCRC[0]); Serial3.write(ReadCRC[1]);
        //Serial3.write(RecievedCRC);

        //Serial3.write((int)ID);
        //Serial3.write((int)lenght);
        //Serial3.write((int)Instruction)

        //Serial3.write(CalcCRC);

        for (size_t i = 0; i < sizeof(CRCArray); i++)
        {
            Serial3.write(CRCArray[i]);
        }

        Serial3.write(ReadCRC[0]); 
        Serial3.write(ReadCRC[1]);
        byte CRC2 = CalcCRC & 0xff;
        byte CRC1 = (CalcCRC >> 8);
        
        Serial3.write(CRC1); 
        Serial3.write(CRC2);
        Serial3.write(Processtime);
        }
        
        if (CalcCRC == RecievedCRC)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            for (size_t i = 0; i < sizeof(ReadData) - 2; i++)
            {
                robotArm::Parameters[i] = Param[i];
            }
            robotArm::MotorID = ID;
            robotArm::Instruction = Instruction;
            return true;
        }
        else
        {
            Serial.println("\nDATA WAS CORRUPTED");
            return false;
        }
    
        
    }
    return false;
}


unsigned short robotArm::CalculateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void robotArm::startMotors()
{
    for (size_t i = 1; i < 7; i++)
    {
        dxl->torqueOff(i);
        dxl->setOperatingMode(i, OP_PWM);

        dxl->torqueOn(i);
    }
}