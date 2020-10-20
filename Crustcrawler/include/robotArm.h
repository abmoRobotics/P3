#pragma once
#include "Dynamixel2Arduino.h"
#define DXL_SERIAL Serial1


class robotArm
{
    
private:
    
    Dynamixel2Arduino *dxl;
   
public:
    robotArm(Dynamixel2Arduino &dxl);
    void startMotors();
    double getTorque(int motorID);
    void setTorque(int motorID, int goalTorque); // Måske ikke lav
    float getPosition(int motorID);             // Position in radians
    void setPosition(int motorID, int16_t goalPos);  // Måske ikke lav
    double getVelocity(int motorID);
    void setVelocity(int motorID, int goalVel);
    void gripperOpen(int GripperL, int GripperR);
    void gripperClose(int GripperL, int GripperR);
};
enum commandList {
     setTorque = 10,
     getTorque,
     getPosition,
     setPosition,
     getVelocity,
     setVelocity
     };