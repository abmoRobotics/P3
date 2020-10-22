#pragma once
#include "SerialClass.h"
#include <vector>

class arduinoCOM
{
public:
    Serial* SP = new Serial("\\\\.\\COM10");    // adjust as needed
    double getTorque() {};
    void setTorque(double goalTorque, char Data[256], int motorID); // Måske ikke lav
    int16_t getPosition(int motorID); // Position in radians
    void setPosition(int16_t goalPos, int motorID); // Måske ikke lav
    double getVelocity(int motorID, char Data[256]);
    void setVelocity(double goalVel, char Data[256], int motorID);
    double getAcceleration() {};
    double setAcceleration() {};
    bool isConnected() { return SP->IsConnected(); };

    
    char Data[256] {};// {kommando, motorid, værdi}
};


