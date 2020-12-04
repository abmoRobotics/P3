#pragma once
#include "SerialClass.h"
#include <vector>

class arduinoCOM
{
public:
    const char* comPort{ "\\\\.\\COM15" };
    Serial* SP = new Serial("\\\\.\\COM15");    // adjust as needed
    void setGripperTorque(int16_t goalTorque, byte motorID, byte direction); // M�ske ikke lav
    void setJointPosition(byte motorID, int16_t goalPos); // M�ske ikke lav
    void setJointVelocity(byte motorID, int16_t goalVel, byte direction);
    bool isConnected() { return SP->IsConnected(); };
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size);
    void serialData(byte motorID, byte Instruction, std::vector<byte>);
    void SelectComPort();

    char Data[256] {};// {kommando, motorid, v�rdi}

    arduinoCOM();
};


