#pragma once
#include "SerialClass.h"
#include <vector>

class Control
{
public:
    Serial* SP = new Serial("\\\\.\\COM12");    // adjust as needed
    double getTorque() {};
    void setTorque(double goalTorque, char Data[256], int motorID); // M�ske ikke lav
    double getPosition(int motorID, char Data[256]); // Position in radians
    void setPosition(double goalPos, char Data[256], int motorID); // M�ske ikke lav
    double getVelocity(int motorID, char Data[256]);
    void setVelocity(double goalVel, char Data[256], int motorID);
    double getAcceleration() {};
    double setAcceleration() {};
    bool isConnected() { return SP->IsConnected(); };

    
    char Data[256] {};// {kommando, motorid, v�rdi}
};


