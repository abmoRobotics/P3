#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>
#include <iostream>
#include "Control.h"

// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
	printf("Welcome to the serial test app!\n\n");

	Control Test;

	

	if (Test.isConnected())
		printf("We're connected");



	//while (Test.isConnected())
	//{

	//int16_t a = (secondByte << 8) + firstByte;
	int16_t ke = -2206;
		
	//}
	//Test.setPosition(Test.getPosition(1, Test.Data) + 100, Test.Data, 1);
	//Test.setPosition(1,Test.Data,1);
	Test.getPosition(1, Test.Data);
	Sleep(2000);
	//Test.setPosition(Test.getPosition(2, Test.Data) + 100, Test.Data, 2);
	//Sleep(2000);

	return 0;
}