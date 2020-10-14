#include <robotArm.h>
#include <Dynamixel2Arduino.h>

robotArm::robotArm(Dynamixel2Arduino &dxl2){
    dxl = &dxl2;
    dxl->begin(57600);
    dxl->setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    startMotors();
}

double robotArm::getTorque(int motorID)
{
    double measuredTorque = dxl->getPresentCurrent(motorID);
    return measuredTorque;
}

void robotArm::setTorque(int motorID, int goalTorque)
{
    dxl->setGoalCurrent(motorID, goalTorque);
} // Måske ikke lav

void robotArm::getPosition(int motorID)
{
    int16_t measuredPos = dxl->getPresentPosition(motorID);
    double radianPos = ((2 * PI / 4095) * measuredPos);
    char firstByte = (byte)measuredPos;         //
    char secondByte = (byte)(measuredPos >> 8); // Shift 8 bit to left
    Serial.write(firstByte);                    // Write first byte representing a number from 0-255
    Serial.write(secondByte);                   // Write second byte representing a number from 256 til noget stort(ca 32000)

} // Position in radians
 // Position in radians

void robotArm::setPosition(int motorID, int16_t goalPos)
{
    dxl->setGoalPosition(motorID, goalPos);
} // Måske ikke lav

double robotArm::getVelocity(int motorID)
{
    double measuredVel = dxl->getPresentVelocity(motorID);
    return measuredVel;
}

void robotArm::setVelocity(int motorID, int goalVel)
{
    dxl->setGoalVelocity(motorID, goalVel);
}

void robotArm::startMotors(){

}