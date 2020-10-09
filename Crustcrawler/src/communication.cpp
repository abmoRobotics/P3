#include "communication.h"
#include "Dynamixel2Arduino.h"
#include <math.h>
    

    double communication::getTorque(int motorID, Dynamixel2Arduino dxl) {
        double measuredTorque = dxl.getPresentCurrent(motorID);

        return measuredTorque;
    }
    void communication::setTorque(int motorID, int goalTorque, Dynamixel2Arduino dxl) {
        dxl.setGoalCurrent(motorID, goalTorque);
    } // Måske ikke lav
    char communication::getPosition(int motorID, Dynamixel2Arduino dxl) {
        double measuredPos = dxl.getPresentPosition(motorID);
        double radianPos = ((2*PI/4095)*measuredPos);
        return measuredPos;
    } // Position in radians

    void communication::setPosition(int motorID, int goalPos, Dynamixel2Arduino dxl) {
        dxl.setGoalPosition(motorID, goalPos);
    } // Måske ikke lav

    double communication::getVelocity(int motorID, Dynamixel2Arduino dxl) {
        double measuredVel = dxl.getPresentVelocity(motorID);
        return measuredVel;
    }

    void communication::setVelocity(int motorID, int goalVel, Dynamixel2Arduino dxl) {
        dxl.setGoalVelocity(motorID, goalVel);
    }


void communication::gripperOpen(int GripperL, int GripperR, Dynamixel2Arduino dxl)
{
  dxl.setGoalPosition(GripperL, 100);
  dxl.setGoalPosition(GripperR, 200);
  dxl.ledOn(GripperL);
  dxl.ledOn(GripperR);
}

void communication::gripperClose(int GripperL, int GripperR, Dynamixel2Arduino dxl)
{
  dxl.setGoalPosition(GripperL, 100);
  dxl.setGoalPosition(GripperR, 200);
  dxl.ledOff(GripperL);
  dxl.ledOff(GripperR);
}
    // double getAcceleration(int motorID) {
    //     double measuredAccel {};
    //     return measuredAccel;
    // };
    // double setAcceleration() {
    //     double value;
    //     return value;
    // };