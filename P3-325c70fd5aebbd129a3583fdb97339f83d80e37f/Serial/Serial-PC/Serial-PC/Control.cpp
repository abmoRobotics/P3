#include "Control.h"


double getTorque()
{

	return 0;
}


void Control::setTorque(double goalTorque, char Data[3], int motorID)
{
	Data[0] = 10;
	Data[1] = motorID;
	Data[2] = goalTorque;
 	SP->WriteData(Data, sizeof(Data));
}; // Måske ikke lav

double Control::getPosition(int motorID, char Data[3])
{
	Data[0] = 12;
	Data[1] = motorID;
	Data[2] = NULL;

	char incomingData[256] = "";			// don't forget to pre-allocate memory
	int dataLength = 255;
	int readResult = 0;


	SP->WriteData(Data, sizeof(Data));

	bool waitForRead = TRUE;
	while (readResult == 0) {
		readResult = SP->ReadData(incomingData, dataLength);
	}
	return incomingData[0];
}; // Position in radians
double setPosition() 
{
	return 0;
}; // Måske ikke lav
double getVelocity() 
{
	return 0;
};
double setVelocity() 
{
	return 0;
};
double getAcceleration() 
{
	return 0;
};
double setAcceleration() 
{
	return 0;
};