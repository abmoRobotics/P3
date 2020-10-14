#pragma once
#include <Dynamixel2Arduino.h>

//namespace communication
//{

    // void getAcceleration(){};
    // void setAcceleration(){};
//} // namespace communication


namespace communication
{


    double getTorque(int motorID, Dynamixel2Arduino dxl);
    void setTorque(int motorID, int goalTorque, Dynamixel2Arduino dxl); // Måske ikke lav
    int16_t getPosition(int motorID, Dynamixel2Arduino &dxl);             // Position in radians
    void setPosition(int motorID, int16_t goalPos, Dynamixel2Arduino &dxl);  // Måske ikke lav
    double getVelocity(int motorID, Dynamixel2Arduino dxl);
    void setVelocity(int motorID, int goalVel, Dynamixel2Arduino dxl);
    void gripperOpen(int GripperL, int GripperR, Dynamixel2Arduino dxl);
    void gripperClose(int GripperL, int GripperR, Dynamixel2Arduino dxl);
    //uint8_t command, motorID;
    //uint16_t value;
   
}
 enum commandList {
     setTorque = 10,
     getTorque,
     getPosition,
     setPosition,
     getVelocity,
     setVelocity
     };

