#pragma once
#include "Dynamixel2Arduino.h"
#define DXL_SERIAL Serial2


class robotArm
{
    
private:
    
    Dynamixel2Arduino *dxl;
    //int MyData[5000][2] = { { 0 } };
    //uint32_t Counter = 0;
    typedef struct sr_data{
        int32_t present_velocity;
        int32_t present_position;
    }  __attribute__((packed)) sr_data_t;
typedef struct sr_data_conv{
        double present_velocity;
        double present_position;
    }  __attribute__((packed)) sr_data_conv_t;
    typedef struct sw_data{
        int32_t goal_PWM;
    } __attribute__((packed)) sw_data_t;
   /*
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
    double calculatePWM(int motorid, float torque, float angularVel, float rot_dir);
    void setPWM(int motorID, float PWM);
    void setTorque2(int motorID, float torque,float angularVel);
    bool dataGatherer();
    double getPositionRad(int motorID);
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
    double calculateMass(int motorID, double Q1, double Q2, double Q3, double Q4);
    double calculateCoriolis(int motorID, double Q1, double Q2, double Q3, double Q4, double DQ1, double DQ2, double DQ3, double DQ4);
    double calculateGravity(int motorID, double Q1, double Q2, double Q3, double Q4);
    double ControlSystem(double ref_Q1, double ref_Q2, double ref_DQ3, double ref_DQ4);
    void MotorConstants(int motorID);
    void SaveData(int Actual, int Ref);
    void PrintData();
    double Read_Data(sr_data_conv_t *ReadData, int size);
    double Write_Data(double tau1, double tau2, double tau3, double tau4);
    double Tester();
};
*/
public:
    char Instruction{};
    int MotorID{};
    byte Parameters[]{};


    robotArm(Dynamixel2Arduino &dxl);
    void startMotors();
    void setGripperTorque(byte motorID, byte goaltorque[]);
    void setJointVelocity(int motorID, byte goalVelocity[]);
    void setJointPositition(int motorID, byte goalPosition[]);

    int16_t getPosition(int motorID);   // Position in ticks
    double getPositionRad(int motorID); // Position in Rads
    double getVelocity(int motorID); 
    double calculatePWM(int motorid, float torque, float angularVel, float Q);
    void setPWM(int motorID, float PWM);
    void setTorque2(int motorID, float torque,float angularVel);
    
    bool dataGatherer();
    unsigned short CalculateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
    double calculateMass(int motorID, double Q1, double Q2, double Q3, double Q4);
    double calculateCoriolis(int motorID, double Q1, double Q2, double Q3, double Q4, double DQ1, double DQ2, double DQ3, double DQ4);
    double calculateGravity(int motorID, double Q1, double Q2, double Q3, double Q4);
    double ControlSystem(double ref_DQ1, double ref_DQ2, double ref_DQ3, double ref_DQ4);
    void MotorConstants(int motorID);
    double Read_Data(sr_data_conv_t *ReadData, int size);
    double Write_Data(double tau1, double tau2, double tau3, double tau4);
    void SaveData(int Actual, int Ref);
    void PrintData();
    double Tester();
};
enum commandList {
     setJointVelocity = 0x10,
     setJointPosition,
     setGripperTorque
     };