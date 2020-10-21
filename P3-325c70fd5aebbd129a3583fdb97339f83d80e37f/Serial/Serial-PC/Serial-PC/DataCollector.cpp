#include "DataCollector.h"
#include <math.h>



//Funktionen kaldes når vi modtager et nyt pose fra myo
void DataCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {

	if (pose == pose.doubleTap) {

		//Reset procent hvis den er over 0
		if (procent > 0) {
			procent = 0;
		}
		//Begynd fistmode hvis procent er 0
		else if (procent == 0) {
			fistModeOn = true;

		}
	}

}

//Funktionen kører hele tiden og tjekker om man dobbeltapper (Funktionen bliver kaldt i main og startes i et nyt multithread)
void DataCollector::fistModeTimer() {
	while (true) {
		Sleep(10);
		if (fistModeOn) { //Når man har dobbeltappet
			Sleep(5000);
			fistModeOn = false;
			std::cout << "LOCKED" << std::endl;
			PlaySound(TEXT("targetLocked.wav"), NULL, SND_SYNC);
		}
	}
}

//Get rawEmg data and put it in a matrix
void DataCollector::GetData(const int8_t* emg) {
	//Modtag data
	for (int i = 0; i < 8; i++)
	{
		emgSamples[i] = emg[i];
	}

	//Index til nyt element
	int index = counter % sampleSize;

	//Flyt rå data til emgData udfra nyt index
	for (int i = 0; i < 8; i++) {
		emgData[index][i] = emgSamples[i];
	}
}

void DataCollector::CalculateAverage() {
	int sum;
	//Udregn genenmsnit for hver pod
	for (int i = 0; i < 8; i++) { //Loop pods
		sum = 0;
		for (int j = 0; j < sampleSize; j++) { //Loop søjle af et enkelt pod
			sum = sum + abs(emgData[j][i]);
		}
		average[i] = sum / sampleSize;
	}

	//Print rå data
	if (showRawEmg) {
		for (int i = 0; i < 8; i++) {
			std::cout << "[" << average[i] << "]";
		}
		std::cout << " " << std::endl;
	}

	//Gennemsnittet bruges til hver bevælgelse
	fistAvg = average[0] + average[3] + average[4] + average[5] + average[6] + average[7];
	upAvg = average[3] + average[4] + average[5];
	downAvg = average[0] + average[4] + average[6] + average[7];
	outAvg = average[5] + average[6];
	inAvg = average[0] + average[3] + average[7];

}

void DataCollector::GetPose() {
	//If statements som finder ud af hvilken bevgælese der bliver lavet

	if (downAvg > downThreshold && average[1] <= 25 && average[2] <= 15 && average[3] <= 15 && !fistModeOn) {
		myoData[1] = 1;
	}
	else if (upAvg > upThreshold && average[1] <= 15 && average[6] <= 20 && average[7] <= 15 && !fistModeOn) {
		myoData[0] = 1;
	}
	else if (inAvg > inThreshold && average[6] <= 12 && !fistModeOn) {
		myoData[3] = 1;
	}
	else if (outAvg > outThreshold && average[0] < 20 && average[1] <= 10 && average[2] <= 10 && average[3] <= 20 && !fistModeOn) {
		myoData[2] = 1;
	}
	//Hvis fist bliver lavet, udregn procent
	else if (fistAvg > fistMinThreshold && fistModeOn) {
		procent = ((float)fistAvg / (float)fistMaxThreshold) * 100;
		//I tilfælde at man kommer over 100%
		if (procent > 100) {
			procent = 100;
		}
	}

}

//Funktionen kaldes når vi modtager nyt emgData (hver 5 millisekund eller noget fast)
void DataCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
	myo->unlock(myo::Myo::unlockHold); //myo bandet unlockes hver 5 millisekund for at sikre at dobbeltap virker på første forsøg)

	//Modtag rå emgData og indsæt i en matrixe
	GetData(emg);

	//Tjek om der er nok data til at udregne gennemsnit (arrayet skal være fyldt, e.x alle 200 elementer)
	if (counter > sampleSize - 1) {
		avgReady = true;
	}

	//Hvis gennemsnit er klar til at blive udregnet
	if (avgReady) {
		CalculateAverage();
	}

	//Reset det data som skal sendes til arduino
	myoData[0] = 0;
	myoData[1] = 0;
	myoData[2] = 0;
	myoData[3] = 0;

	//Find pose efter setup er færdig
	if (finishedSetup) {
		GetPose();
		//if(myoData[0])
		//{ //op 
		//}
		//else if(myoData[1])
		//{ //Ned
		//}
		//else if (myoData[2])
		//{		
		//}
		//else if (myoData[3])
		//{
		//	//out
		//}
		//else if (myoData[4])
		//{
		//	//in
		//}
		
	}

	//Indsæt procent i data som skal sendes
	myoData[4] = (int)procent;

	//Print myoData i konsol
	if (finishedSetup && showMyoData)
		std::cout << "[" << myoData[0] << "," << myoData[1] << "," << myoData[2] << "," << myoData[3] << "," << myoData[4] << "," << myoData[5] << "," << myoData[6] << "," << myoData[7] << "]" << std::endl;
	else if (finishedSetup && showPose)
		std::cout << "[" << myoData[0] << "," << myoData[1] << "," << myoData[2] << "," << myoData[3] << "," << myoData[4] << "]" << std::endl;

	//Counteren for emgData stiger
	counter++;
}


using std::atan2;
using std::asin;
using std::sqrt;
using std::max;
using std::min;
//Funktionen kaldes når et nyt orientation er givet fra myoband
void DataCollector::onOrientationData(myo::Myo* myo, uint64_t timestap, const myo::Quaternion<float>& rotation) {

	int newRoll = 0;
	int newPitch = 0;
	int newYaw = 0;

	//Udregn euler angles(roll, pitch, yaw) udfra quaternion

	float rollRad = atan2(2.0f * (rotation.w() * rotation.x() + rotation.y() * rotation.z()), //quik maffs
		1.0f - 2.0f * (rotation.x() * rotation.x() + rotation.y() * rotation.y()));
	float pitchRad = asin(max(-1.0f, min(1.0f, 2.0f * (rotation.w() * rotation.y() - rotation.z() * rotation.x()))));
	float yawRad = atan2(2.0f * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
		1.0f - 2.0f * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));

	//Convert to degrees
	roll = (int)(rollRad * 180 / M_PI);
	pitch = (int)(pitchRad * 180 / M_PI);
	yaw = (int)(yawRad * 180 / M_PI);

	//Udregn den kalibreret roll pitch yaw
	newRoll = roll - startRoll;
	newPitch = pitch - startPitch;
	newYaw = yaw - startYaw;

	//Send data til arduino
	myoData[5] = newRoll;
	myoData[6] = newPitch;
	myoData[7] = newYaw;

	if (showOrientation && finishedSetup) {
		std::cout << newRoll << "," << newPitch << "," << newYaw << std::endl;
	}

}

//Sender data til arduino hele tiden 
void DataCollector::ArduinoThread() {
	while(1) {
		if (finishedSetup) {
			sendOrientationToArduino();	
		}
		Sleep(15);
	}
}

//Sender orientationen til arduino
void DataCollector::sendOrientationToArduino()
{

	int nullmotor1{ 1250 }; //Ticks når motoren er i nul position
	int fullmotor1{ 2300 }; //Ticks når motoren er i maks position
	int middlemotor1 = (fullmotor1 - nullmotor1) / 2 + nullmotor1; //Ticks når motoren er midtvejs
	int fullmotor1Deg = 90; //Maks grader man kan bevæge armen

	int nullmotor2{ 1350 }; //Ticks når motoren er i nul position
	int fullmotor2{ 2750 }; //Ticks når motoren er i maks position
	int middlemotor2 = (fullmotor2 - nullmotor2) / 2 + nullmotor2; //Ticks når motoren er midtvejs
	int fullmotor2Deg = 100; //Maks grader man kan bevæge armen

	int16_t goalPosPitch = (fullmotor1 - middlemotor1) / (fullmotor1Deg)*myoData[6] + middlemotor1; //ax+b funktion, udregner ticks ud fra pitch degrees
	int16_t goalPosRoll = (fullmotor2 - middlemotor2) / (fullmotor2Deg)*myoData[5] + middlemotor2; //ax+b funktion, udregner ticks ud fra roll degrees

	//Send ny position til arduinoen
	
	Arduino.setPosition(goalPosPitch, 1);
	Sleep(10);
	Arduino.setPosition(goalPosRoll, 2);
	



}

//Start threads when constructed
void DataCollector::startThreads() {
	std::thread t(&DataCollector::setupMyo, this);
	std::thread t2(&DataCollector::fistModeTimer, this);
	std::thread t3(&DataCollector::ArduinoThread, this);
	t2.join();
}


//Kalibrere setup (bliver kaldt af main funktionen når)
void DataCollector::setupMyo() {


	if (!Arduino.isConnected())
	{
		std::cout << "Arduino is not connected" << std::endl;
		Sleep(500);
	}

	std::cout << "Arduino is connected!" << std::endl;

	std::cout << "Starting setup" << std::endl;

	//Vent indtil gennemsnittet af hver pod kan blive udregnet
	while (avgReady == false) {
		Sleep(100);
	}

	/// 
	/// DEBUG
	/// 

	char input;
	std::cout << "Show raw EMG data [y/n]" << std::endl; std::cin >> input;	
	if (input == 'y') showRawEmg = true;

	std::cout << "Show myoData [y/n]" << std::endl; std::cin >> input;
	if (input == 'y') showMyoData = true;

	std::cout << "Show pose data [y/n]" << std::endl; std::cin >> input;
	if (input == 'y') showPose = true;

	std::cout << "Show orientation [y/n]" << std::endl; std::cin >> input;
	if (input == 'y') showOrientation = true;

	//Skip setup
	std::cout << "Skip calibration setup? [y/n]" << std::endl;
	std::cin >> input;

	if(input == 'n'){

		std::cout << "Welcome to Myoband Calibration Setup" << std::endl;
		system("pause");

		/// 
		/// UP POSE
		/// 

		std::cout << "Perform Up pose" << std::endl;
		Sleep(3000);

		//press any key to resume
		system("pause");

		//Get up average
		upThreshold = average[3] + average[4] + average[5];
		std::cout << "Up: " << upThreshold << std::endl;

		//Calculate threshold
		upThreshold = upThreshold - 20;
		std::cout << "Up threshold: " << upThreshold << std::endl;

		Sleep(1000);

		///
		/// DOWN POSE
		///

		std::cout << "Perform Down pose" << std::endl;
		Sleep(3000);

		//press any key to resume
		system("pause");

		//Get down average
		downThreshold = average[0] + average[4] + average[6] + average[7];
		std::cout << "Down: " << downThreshold << std::endl;

		//Calculate threshold
		downThreshold = downThreshold - 20;
		std::cout << "Down threshold: " << downThreshold << std::endl;
		Sleep(1000);

		///
		/// OUT POSE
		///

		std::cout << "Perform Out pose" << std::endl;
		Sleep(3000);

		//Press any key to resume
		system("pause");

		//Get out average
		outThreshold = average[5] + average[6];
		std::cout << "Out: " << outThreshold << std::endl;

		//Calculate threshold
		outThreshold = outThreshold - 12;
		std::cout << "Out threshold: " << outThreshold << std::endl;
		Sleep(1000);

		///
		/// IN POSE
		///

		std::cout << "Perform In pose" << std::endl;
		Sleep(3000);

		//Press any key to resume
		system("pause");

		//Get In average
		inThreshold = average[0] + average[3] + average[7];
		std::cout << "In: " << inThreshold << std::endl;

		//Calculate threshold
		inThreshold = inThreshold - 12;
		std::cout << "In threshold: " << inThreshold << std::endl;
		Sleep(1000);

		///
		/// MIN FIST
		///

		std::cout << "Perform Min Fist" << std::endl;
		Sleep(3000);

		//Press any key to resume
		system("pause");

		// Get Min fist average
		fistMinThreshold = average[0] + average[3] + average[4] + average[5] + average[6] + average[7];
		std::cout << "Fist: " << fistMinThreshold << std::endl;
		Sleep(1000);

		///
		/// MAX FIST
		///

		std::cout << "Perform Max Fist" << std::endl;
		Sleep(3000);

		//Press any key to resume
		system("pause");

		// Get Max fist average
		fistMaxThreshold = average[0] + average[3] + average[4] + average[5] + average[6] + average[7];
		std::cout << "Max Fist: " << fistMaxThreshold << std::endl;
		Sleep(1000);
		
	}

	/// 
	/// ORIENTATION
	/// 

	std::cout << "Initialize Orientation Setup" << std::endl;
	Sleep(2000);
	std::cout << "- Move arm to factory settings -" << std::endl;

	system("pause");
	std::cout << "Orientation values are: " << std::endl;

	startRoll = roll;
	std::cout << "roll: " << startRoll << std::endl;

	startPitch = pitch;
	std::cout << "Pitch: " << startPitch << std::endl;

	startYaw = yaw;
	std::cout << "Yaw: " << startYaw << std::endl;

	/// 
	/// COMPLETE
	/// 
	std::cout << "Calibration completed" << std::endl;
	Sleep(1000);
	system("pause");

	//Sæt setup værdien til true
	finishedSetup = true;

}
