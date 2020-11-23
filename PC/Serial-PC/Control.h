#pragma once
#include "SerialClass.h"
#include <vector>

class arduinoCOM
{
public:
    Serial* SP = new Serial("COM7");    // adjust as needed
    double getTorque() {};
    void setTorque(int16_t goalTorque, byte motorID, byte direction); // M�ske ikke lav
    int16_t getPosition(int motorID); // Position in radians
    void setPosition(int16_t goalPos, byte motorID); // M�ske ikke lav
    double getVelocity(int motorID, char Data[256]);
    void setVelocity(int16_t goalVel, byte motorID);
    double getAcceleration() {};
    double setAcceleration() {};
    bool isConnected() { return SP->IsConnected(); };
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size);
    void serialData(byte motorID, byte Instruction, std::vector<byte>);

    char Data[256] {};// {kommando, motorid, v�rdi}
};


