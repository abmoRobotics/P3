#pragma once
#include "Dynamixel2Arduino.h"
#define DXL_SERIAL Serial1


class robotArm
{
    
private:
    
    Dynamixel2Arduino *dxl;
    
   
public:
    byte Instruction{};
    byte MotorID{};
    byte Parameters[]{};


    robotArm(Dynamixel2Arduino &dxl);
    void startMotors();
    double getTorque(int motorID);
    void setTorque(int motorID, float goalTorque); // Måske ikke lav
    int16_t getPosition(int motorID);             // Position in radians
    void setPosition(int motorID, int16_t goalPos);  // Måske ikke lav
    double getVelocity(int motorID);
    void setVelocity(int motorID, int goalVel);
    double calculatePWM(int motorid, float torque);
    bool dataGatherer();
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
};
enum commandList {
     setTorque = 10,
     getTorque,
     getPosition,
     setPosition,
     getVelocity,
     setVelocity
     };