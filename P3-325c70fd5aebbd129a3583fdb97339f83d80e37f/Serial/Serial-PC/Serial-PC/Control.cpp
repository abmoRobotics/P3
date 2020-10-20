#include "control.h"
#include <iostream>
#include <string.h>
#include <sstream>

//Ikke lavet (den laver emil og anton i arduino)
double getTorque()
{
	
	return 0;
}


void arduinoCOM::setTorque(double goalTorque, char Data[256], int motorID)
{
	Data[0] = 10;
	Data[1] = motorID;
	Data[2] = goalTorque;

 	SP->WriteData(Data, sizeof(Data));
}; // Måske ikke lav

int16_t arduinoCOM::getPosition(int motorID, char Data[256])
{
	Data[0] = 12;
	Data[1] = motorID;
	Data[2] = NULL;
	Data[3] = NULL;

	char incomingData[256] = "";		// don't forget to pre-allocate memory
	int dataLength = 255;
	int readResult = 0;
	
	
	SP->WriteData(Data, sizeof(Data));

	bool waitForRead = TRUE;
	while (readResult == 0) {
		readResult = SP->ReadData(incomingData, dataLength);
	}

	int16_t firstByte = incomingData[0];
	int16_t secondByte = (incomingData[1] << 8);	// Shift data 8 bits to the left. 
	int16_t position = firstByte + secondByte;		// Add the value of the two bytes
	
	return position;
}; // Position in radians
void arduinoCOM::setPosition(int16_t goalPos,int motorID)
{
	char Data[256];
	std::ostringstream sstream;
	sstream << goalPos;
	std::string varAsString = sstream.str();
	Data[0] = 13;
	Data[1] = motorID;
	Data[2] = (byte)goalPos;
	Data[3] = (byte)(goalPos >> 8);

	//std::cout << goalPos << std::endl;
	SP->WriteData(Data, sizeof(Data));
	
}; // Måske ikke lav
double arduinoCOM::getVelocity(int motorID, char Data[256])
{
	Data[0] = 14;
	Data[1] = motorID;
	Data[2] = NULL;
	Data[3] = NULL;
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
void arduinoCOM::setVelocity(double goalVel, char Data[256], int motorID)
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