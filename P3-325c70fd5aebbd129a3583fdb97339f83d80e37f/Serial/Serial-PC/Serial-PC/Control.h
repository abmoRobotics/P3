#pragma once
#include "SerialClass.h"


class Control
{
public:
    Serial* SP = new Serial("\\\\.\\COM11");    // adjust as needed
    double getTorque() {};
    void setTorque(double goalTorque, char Data[3], int motorID); // Måske ikke lav
    double getPosition(int motorID, char Data[3]); // Position in radians
    double setPosition() {}; // Måske ikke lav
    double getVelocity() {};
    double setVelocity() {};
    double getAcceleration() {};
    double setAcceleration() {};
    bool isConnected() { return SP->IsConnected(); };

    
    char Data[3] = {1, 1, 0};
};


