#pragma once
#include "SerialClass.h"


class Control
{
public:
    Serial* SP = new Serial("COM7");    // adjust as needed
    double getTorque() {};
    void setTorque(double goalTorque, char Data[3], int motorID); // Måske ikke lav
    double getPosition(int motorID, char Data[3]); // Position in radians
    void setPosition(double goalPos, char Data[3], int motorID); // Måske ikke lav
    double getVelocity(int motorID, char Data[3]);
    void setVelocity(double goalVel, char Data[3], int motorID);
    double getAcceleration() {};
    double setAcceleration() {};
    bool isConnected() { return SP->IsConnected(); };

    
    char Data[3] = {1, 1, 0};
};


