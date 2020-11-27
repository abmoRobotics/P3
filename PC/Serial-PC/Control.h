#pragma once
#include "SerialClass.h"
#include <vector>

class arduinoCOM
{
public:
    const char* comPort{ "COM8" };
    Serial* SP = new Serial(comPort);    // adjust as needed
    void setGripperTorque(int16_t goalTorque, byte motorID, byte direction); // Måske ikke lav
    void setJointPosition(int16_t goalPos, byte motorID); // Måske ikke lav
    void setJointVelocity(int16_t goalVel, byte motorID, byte direction);
    bool isConnected() { return SP->IsConnected(); };
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size);
    void serialData(byte motorID, byte Instruction, std::vector<byte>);
    void SelectComPort();

    char Data[256] {};// {kommando, motorid, værdi}

    arduinoCOM();
};


