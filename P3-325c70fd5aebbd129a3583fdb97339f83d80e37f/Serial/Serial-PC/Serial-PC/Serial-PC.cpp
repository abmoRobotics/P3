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



	while (Test.isConnected())
	{

		Test.setTorque(10, Test.Data, 1);
		Sleep(5000);
		Test.setTorque(20, Test.Data, 2);
		Sleep(5000);
		std::cout << Test.getPosition(1, Test.Data) << std::endl;
		Sleep(5000);
		std::cout << Test.getPosition(2, Test.Data) << std::endl;
		
		
		
		/*SP->WriteData(Test.Data, sizeof(Test.Data));
		Sleep(5000);
		Test.Data[1]++;
		SP->WriteData(Test.Data, sizeof(Test.Data));
		Sleep(5000);
		Test.Data[0] = 2;
		Test.Data[1] = 1;
		SP->WriteData(Test.Data, sizeof(Test.Data));
		Sleep(5000);
		Test.Data[1]++;
		SP->WriteData(Test.Data, sizeof());
		Sleep(5000);*/




	}
	return 0;
}