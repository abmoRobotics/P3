#pragma once
#include "SerialClass.h"
#include <vector>

class arduinoCOM
{
public:
    Serial* SP = new Serial("COM7");    // adjust as needed
    double getTorque() {};
    void setTorque(double goalTorque, char Data[256], int motorID); // M�ske ikke lav
    int16_t getPosition(int motorID, char Data[256]); // Position in radians
    void setPosition(int16_t goalPos, char Data[256], int motorID); // M�ske ikke lav
    double getVelocity(int motorID, char Data[256]);
    void setVelocity(double goalVel, char Data[256], int motorID);
    double getAcceleration() {};
    double setAcceleration() {};
    bool isConnected() { return SP->IsConnected(); };

    
    char Data[256] {};// {kommando, motorid, v�rdi}
};


