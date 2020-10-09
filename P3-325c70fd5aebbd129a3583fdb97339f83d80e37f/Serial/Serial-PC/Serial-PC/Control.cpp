#include "Control.h"
#include <iostream>

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
}; // M�ske ikke lav

double Control::getPosition(int motorID, char Data[3])
{
	Data[0] = 12;
	Data[1] = motorID;
	Data[2] = NULL;

	char incomingData[256] = "";		// don't forget to pre-allocate memory
	int dataLength = 255;
	int readResult = 0;


	SP->WriteData(Data, sizeof(Data));

	bool waitForRead = TRUE;
	int i{};
	while (readResult == 0) {
		readResult = SP->ReadData(incomingData, dataLength);	
	}
	incomingData[readResult] = 0;
	int ai{};
	sscanf_s(incomingData, "%d", &ai);
	std::cout << ai << std::endl;
	
	
	return ai;
}; // Position in radians
void Control::setPosition(double goalPos, char Data[3], int motorID)
{
	Data[0] = 'D';
	Data[1] = motorID;
	Data[2] = goalPos;
	SP->WriteData(Data, sizeof(Data));
}; // M�ske ikke lav
double Control::getVelocity(int motorID, char Data[3])
{
	Data[0] = 14;
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
	printf("%c", incomingData[0]);
	return incomingData[0];
};
void Control::setVelocity(double goalVel, char Data[3], int motorID)
{
	Data[0] = 13;
	Data[1] = motorID;
	Data[2] = goalVel;
	SP->WriteData(Data, sizeof(Data));
};
double getAcceleration() 
{
	return 0;
};
double setAcceleration() 
{
	return 0;
};