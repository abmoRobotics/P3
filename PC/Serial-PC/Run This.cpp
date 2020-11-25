#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>
#include <iostream>
#include "Control.h"
#include <array>
#include <math.h>
#include <sstream>
#include <stdexcept>
#include <string>
#define _USE_MATH_DEFINES
#include<thread>

//Myo include
#include <myo/myo.hpp>

//Include class
#include "DataCollectorFilter.h"
#define pi = 3.14159





//Run hub funktionen som gør at myobandet sender data ud i 100Hz. (Startes af main i et nyt thread)
myo::Hub hub("com.example.emg-data-sample");
void runHub() {
	while (1) {
		// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
		// In this case, we wish to update our display 50 times a second, so we run for 1000/20 milliseconds.
		hub.run(1000 / 20);
	}
}


// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
	
	try {
		std::cout << "Attempting to find a Myo..." << std::endl;

		//myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		/*if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}*/

		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

		//myo->setStreamEmg(myo::Myo::streamEmgEnabled);


		DataCollector collector;
		hub.addListener(&collector);
		std::thread th1(runHub);
		collector.startThreads();
	
		
		//Block main until th1 has stopped (will never stop >:) )
		th1.join();

		return 0;

	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}


