#pragma once
#include "Dynamixel2Arduino.h"
#define DXL_SERIAL Serial1


class robotArm
{
    
private:
    
    Dynamixel2Arduino *dxl;
    //int MyData[5000][2] = { { 0 } };
    //uint32_t Counter = 0;
    
   
public:
    char Instruction{};
    int MotorID{};
    byte Parameters[]{};


    robotArm(Dynamixel2Arduino &dxl);
    void startMotors();
    double getTorque(int motorID);
    void setTorque(byte motorID, byte goaltorque_ptr[]); // Måske ikke lav
    void setGripperTorque(byte motorID, byte goaltorque[]);
    int16_t getPosition(int motorID);             // Position in radians
    void setPosition(int motorID, int16_t goalPos);  // Måske ikke lav
    double getVelocity(int motorID);
    void setVelocity(int motorID, byte goalVel_ptr[]);
    double calculatePWM(int motorid, float torque, float angularVel, float Q);
    void setPWM(int motorID, float PWM);
    void setTorque2(int motorID, float torque,float angularVel);
    bool dataGatherer();
    double getPositionRad(int motorID);
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
    double calculateMass(int motorID, double Q1, double Q2, double Q3, double Q4);
    double calculateCoriolis(int motorID, double Q1, double Q2, double Q3, double Q4, double DQ1, double DQ2, double DQ3, double DQ4);
    double calculateGravity(int motorID, double Q1, double Q2, double Q3, double Q4);
    double ControlSystem(double ref_DQ1, double ref_DQ2, double ref_DQ3, double ref_DQ4);
    void MotorConstants(int motorID);
    void SaveData(int Actual, int Ref);
    void PrintData();
};
enum commandList {
     setTorque = 0x10,
     getTorque,
     getPosition,
     setPosition,
     getVelocity,
     setVelocity,
     setGripperTorque
     };