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
    int32_t getPosition(int motorID);             // Position in radians
    void setPosition(int motorID, int16_t goalPos);  // Måske ikke lav
    double getVelocity(int motorID);
    void setVelocity(int motorID, int goalVel);
    double calculatePWM(int motorid, float torque);
    void setPWM(int motorID, float PWM);
    bool dataGatherer();
    double getPositionRad(int motorID);
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
    double calculateMass(int motorID, double Q1, double Q2, double Q3, double Q4);
    double calculateCoriolis(int motorID, double Q1, double Q2, double Q3, double Q4, double DQ1, double DQ2, double DQ3, double DQ4);
    double calculateGravity(int motorID, double Q1, double Q2, double Q3, double Q4);
    double ControlSystem(double ref_DQ1, double ref_DQ2, double ref_DQ3, double ref_DQ4);
    void MotorConstants(int motorID);
};
enum commandList {
     setTorque = 10,
     getTorque,
     getPosition,
     setPosition,
     getVelocity,
     setVelocity
     };