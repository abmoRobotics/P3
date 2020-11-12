#include "DataCollectorFilter.h"
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

//Get rawEmg data and put it in the array: emgSamples
void DataCollector::getData(const int8_t* emg) {
	//Modtag data
	for (int i = 0; i < 8; i++)
	{
		previousRawEmg[i] = rawEmg[i]; //Gem tidligere emg data
		rawEmg[i] = emg[i]; //Gem nuværende emg data
	}

	if (finishedSetup && showRawData) {
		for (int i = 0; i < 8; i++) {
			std::cout << "[" << rawEmg[i] << "]";
		}
		std::cout << " " << std::endl;
	}

	//std::cout << " raw: " << rawEmg[4] << std::endl;
	//std::cout << " prev: " << previousRawEmg[4] << std::endl;

}

//Filtrerer rå emg data ved brug af en transfer function
void DataCollector::applyFilter() {

	//Forarbejde
	for (int i = 0; i < 8; i++)
	{
		previousFilteredEmg[i] = filteredEmg[i];
	}
	
	//	//Filtrering
	for(int i = 0; i < 8; i++){
		//u(t) = (samplingstid(Ts) / tidskonstant (T) * "forrige rå data") - (samplingstid(Ts) / tidskonstant (T) * "vægtet forrige filtered emg" + "forrige filtered emg") 
		filteredEmg[i] = ((Ts / T) * abs(previousRawEmg[i])) - ((Ts / T) * previousFilteredEmg[i]) + previousFilteredEmg[i];
	}


	//Print rå data
	if (showFilteredData && finishedSetup) {
		for (int i = 0; i < 8; i++) {
			std::cout << "[" << (int)filteredEmg[i] << "]";
		}
		std::cout << " " << std::endl;
	}

	//Den filtreret data bruges til hver bevælgelse
	fistValue = filteredEmg[0] + filteredEmg[3] + filteredEmg[4] + filteredEmg[5] + filteredEmg[6] + filteredEmg[7];
	upValue = filteredEmg[3] + filteredEmg[4] + filteredEmg[5];
	downValue = filteredEmg[0] + filteredEmg[4] + filteredEmg[6] + filteredEmg[7];
	outValue = filteredEmg[5] + filteredEmg[6];
	inValue = filteredEmg[0] + filteredEmg[3] + filteredEmg[7];

	//std::cout << upValue << std::endl;

}

//Sætter de første 4 Myodata, afhængig af pose
void DataCollector::getPose() {
	//If statements som finder ud af hvilken bevgælese der bliver lavet

	if (downValue > downThreshold && filteredEmg[1] <= 25 && filteredEmg[2] <= 15 && filteredEmg[3] <= 15 && !fistModeOn) {
		myoData[1] = 1;
	}
	else if (upValue > upThreshold && filteredEmg[1] <= 25 && filteredEmg[6] <= 25 && filteredEmg[7] <= 25 && !fistModeOn) {
		myoData[0] = 1;
	}
	else if (inValue > inThreshold && filteredEmg[6] <= 12 && !fistModeOn) {
		myoData[3] = 1;
	}
	else if (outValue > outThreshold && filteredEmg[0] < 20 && filteredEmg[1] <= 10 && filteredEmg[2] <= 10 && filteredEmg[3] <= 20 && !fistModeOn) {
		myoData[2] = 1;
	}
	//Hvis fist bliver lavet, udregn procent
	else if (fistValue > fistMinThreshold && fistModeOn) {
		procent = ((float)fistValue / (float)fistMaxThreshold) * 100;
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

	//Modtag rå emgData og indsæt i en matrixe, samt flyt tidligere data i en anden matrix
	getData(emg);

	//Filtrerer rå emg data
	applyFilter();
	
	//Reset det data som skal sendes til arduino
	myoData[0] = 0;
	myoData[1] = 0;
	myoData[2] = 0;
	myoData[3] = 0;

	//Find pose efter setup er færdig
	if (finishedSetup) {
		getPose();
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

	//Send data til TXT fil.(Python læser denne fil)
	if (counter % 100 == 1) { //1 gange i sekundet. 100/100 = 1
		//dataHandler.SaveData(filteredEmg, "AVGdata.txt");
	}

}

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
void DataCollector::arduinoThread() {
	while(1) {
		if (finishedSetup) {
			sendOrientationToArduino();	
			sendPoseToArduino();
		}
		Sleep(15);
	}
}

//Sender pose til arduino (up, down, out, in)
void DataCollector::sendPoseToArduino()
{
	if (myoData[0] == 1)
	{
		int pos = Arduino.getPosition(3) - 20;
		std::cout << pos << std::endl;
		Sleep(50);
		Arduino.setPosition(pos,3);
	}
	else if (1)
	{
		int16_t pos2 = Arduino.getPosition(3);
		std::cout << "POS2:"<<  pos2 << std::endl;
		Sleep(50);
		int16_t tete = 1700;
		Arduino.setPosition((int16_t)1700, 3);
	}
	Sleep(50);
}

//Sender orientationen til arduino
void DataCollector::sendOrientationToArduino()
{

	int minmotor1{ 2300 }; //Ticks når motoren er i nul position
	int maxmotor1{ 1250 }; //Ticks når motoren er i maks position
	int zeromotor1 = minmotor1;
	//int zeromotor1 = (maxmotor1 - minmotor1) / 2 + minmotor1; //Ticks når motoren er midtvejs
	int fullmotor1Deg = 90; //Maks grader man kan bevæge armen

	int minmotor2{ 2750 }; //Ticks når motoren er i nul position
	int maxmotor2{ 1350 }; //Ticks når motoren er i maks position
	int zeromotor2 = (maxmotor2 - minmotor2) / 2 + minmotor2; //Ticks når motoren er midtvejs
	int fullmotor2Deg = 100; //Maks grader man kan bevæge armen

	int16_t goalPosPitch = (maxmotor1 - zeromotor1) / (fullmotor1Deg)*myoData[6] + zeromotor1; //ax+b funktion, udregner ticks ud fra pitch degrees
	int16_t goalPosRoll = (maxmotor2 - zeromotor2) / (fullmotor2Deg)*myoData[5] + zeromotor2; //ax+b funktion, udregner ticks ud fra roll degrees

	//Send ny position til arduinoen
	
	Arduino.setPosition(goalPosPitch, 1);
	Sleep(50);
	Arduino.setPosition(goalPosRoll, 2);
	Sleep(50);

}

//Start threads when constructed
void DataCollector::startThreads() {
	std::thread t(&DataCollector::setupMyo, this);
	std::thread t2(&DataCollector::fistModeTimer, this);
	std::thread t3(&DataCollector::arduinoThread, this);
	t2.join();
}

//Kalibrere setup (bliver kaldt af main funktionen når)
void DataCollector::setupMyo() {

	//Loop indtil arduino er connected
	if (!Arduino.isConnected())
	{
		std::cout << "Arduino is not connected" << std::endl;
		Sleep(500);
	}

	std::cout << "Arduino is connected!" << std::endl;

	std::cout << "Starting setup" << std::endl;


	/// 
	/// DEBUG
	/// 

	//Her kan brugeren bestemme hvilken data de vil vise i terminalen
	char input;

	std::cout << "Show raw data [y/n]" << std::endl; std::cin >> input;	
	if (input == 'y') showRawData = true;

	std::cout << "Show filtered data [y/n]" << std::endl; std::cin >> input;	
	if (input == 'y') showFilteredData = true;

	std::cout << "Show myoData [y/n]" << std::endl; std::cin >> input;
	if (input == 'y') showMyoData = true;

	//Skip setup
	std::cout << "Run calibration setup? [y/n]" << std::endl;
	std::cin >> input;

	if(input == 'y'){

		float temp = T;
		T = T_calibration; //Sæt Tidskonstanten lig før kalibrering (Filteringen tager længere tid, men spiker ikke så meget)

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
		upThreshold = filteredEmg[3] + filteredEmg[4] + filteredEmg[5];
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
		downThreshold = filteredEmg[0] + filteredEmg[4] + filteredEmg[6] + filteredEmg[7];
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
		outThreshold = filteredEmg[5] + filteredEmg[6];
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
		inThreshold = filteredEmg[0] + filteredEmg[3] + filteredEmg[7];
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
		fistMinThreshold = filteredEmg[0] + filteredEmg[3] + filteredEmg[4] + filteredEmg[5] + filteredEmg[6] + filteredEmg[7];
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
		fistMaxThreshold = filteredEmg[0] + filteredEmg[3] + filteredEmg[4] + filteredEmg[5] + filteredEmg[6] + filteredEmg[7];
		std::cout << "Max Fist: " << fistMaxThreshold << std::endl;
		Sleep(1000);

		T = temp; //Reset T tilbage igen
		
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
