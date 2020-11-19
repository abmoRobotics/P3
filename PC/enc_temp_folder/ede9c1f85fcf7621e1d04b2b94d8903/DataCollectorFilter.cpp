#include "DataCollectorFilter.h"
#include <math.h>

//Funktionen kaldes n�r vi modtager et nyt pose fra myo
void DataCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {

	if (pose == pose.doubleTap) {

		//Reset procent hvis den er over 0
		if (procent > 0) {
			procent = 0;
		}
		//Begynd fistmode hvis procent er 0
		else if (procent == 0) {
			fistControlOn = true;

		}
	}

}

//Funktionen k�rer hele tiden og tjekker om man dobbeltapper (Funktionen bliver kaldt i main og startes i et nyt multithread)
void DataCollector::fistModeTimer() {
	while (true) {
		Sleep(10);
		if (fistControlOn) { //N�r man har dobbeltappet
			Sleep(5000);
			fistControlOn = false;
			std::cout << "LOCKED" << std::endl;
			PlaySound(TEXT("targetLocked.wav"), NULL, SND_SYNC);
		}
	}
}
// F�r rawEmg data og s�tter det ind i et array: emgSamples
void DataCollector::getData(const int8_t* emg) {
	//Modtag data
	for (int i = 0; i < 8; i++)
	{
		previousRawEmg[i] = rawEmg[i]; //Gem tidligere emg data
		rawEmg[i] = emg[i]; //Gem nuv�rende emg data
	}

	//Printer data i terminalen
	if (finishedSetup && showRawData) {
		for (int i = 0; i < 8; i++) {
			std::cout << "[" << rawEmg[i] << "]";
		}
		std::cout << " " << std::endl;
	}

}

//Filtrerer r� emg data ved brug af en transfer function
void DataCollector::applyFilter() {

	//Forarbejde
	for (int i = 0; i < 8; i++)
	{
		previousFilteredEmg[i] = filteredEmg[i];
	}
	
	//	//Filtrering
	for(int i = 0; i < 8; i++){
		//u(t) = (samplingstid(Ts) / tidskonstant (T) * "forrige r� data") - (samplingstid(Ts) / tidskonstant (T) * "v�gtet forrige filtered emg" + "forrige filtered emg") 
		filteredEmg[i] = ((Ts / T) * abs(previousRawEmg[i])) - ((Ts / T) * previousFilteredEmg[i]) + previousFilteredEmg[i];
	}


	//Print filtreret data
	if (showFilteredData && finishedSetup) {
		for (int i = 0; i < 8; i++) {
			std::cout << "[" << (int)filteredEmg[i] << "]";
		}
		std::cout << " " << std::endl;
	}

}

//S�tter de f�rste 4 Myodata, afh�ngig af pose
void DataCollector::getPose() {

	//Tjekker hvilken pose der bliver lavet
	bool movements[4] = {1, 1, 1, 1}; //[up, down, out, in]
	for (int i = 0; i < 4; i++) { // K�rer igennem alle movements
		for (int j = 0; j < 8; j++) { // K�rer igennem alle filteret pods
			if (filteredEmg[j] < MinPods[i][j] || filteredEmg[j] > MaxPods[i][j]) { //Hvis pod IKKE er i invervallet
				movements[i] = false;
			}
		}
	}

	//Fixer s� flere poses ikke kan v�re aktive p� samme tid
	//K�rer i gennem alle poses, tjekker om de er aktive, og deaktiverer resten
	for (int i = 0; i < 4; i++) {
		if (movements[i]) {
			for (int j = 0; j < 4; j++) { //Reset alle movements til 0
				movements[j] = false;
			}
			movements[i] = true; //S�t den aktive pose til true igen
		}
	}

	//Fist
	if (fistControlOn) {
		//S�tter alle "movements" til at v�re 0	 
		for (int i = 0; i < 4; i++) {
			movements[i] = 0;
		}


		//Regner summen af alle pods ud
		int sum = 0;
		int sumMin = 0;
		int sumMax = 0;
		for (int i = 0; i < 8; i++) {
			sum = filteredEmg[i] + sum; //Sum af nuv�rende pods
			sumMin = fistMin[i] + sumMin; //Sum af kalibreret min fist
			sumMax = fistMax[i] + sumMax; //Sum af den kalibreret max Fist
		} 

		//Regner procent
		procent = (((float)sum - (float)sumMin) / ((float)sumMax - (float)sumMin)) * 100;
		//I tilf�lde at man kommer over 100%
		if (procent > 100) {
			procent = 100;
		}
		//i tilf�lde at man kommer under 0%
		else if(procent < 0){
			procent = 0;
		}
	}



	//Prints
	if (showIntervalData) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 8; j++) {
				std::cout << i << ": " << movements[i];
				std::cout << " EMG: ";
				for (int k = 0; k < 8; k++) {
					std::cout << " [" << (int)filteredEmg[k] << "]" << "(" << MaxPods[i][k] << " " << MinPods[i][k] << ")";
				}
				std::cout << "" << std::endl;
			}
		}
	}
	//Prints
	if (showPoses) {
		for (int i = 0; i < 4; i++) {
			std::cout << " [" << movements[i] << "]";
		}
		std::cout << " [" << (int)procent << "%]" << std::endl;
	}

}

//Funktionen kaldes n�r vi modtager nyt emgData (hver 5 millisekund eller noget fast)
void DataCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
	myo->unlock(myo::Myo::unlockHold); //myo bandet unlockes hver 5 millisekund for at sikre at dobbeltap virker p� f�rste fors�g)

	//Modtag r� emgData og inds�t i en matrice, samt flyt tidligere data i en anden matrice
	getData(emg);

	//Filtrerer r� emg data
	applyFilter();
	
	//Reset det data som skal sendes til arduino
	myoData[0] = 0;
	myoData[1] = 0;
	myoData[2] = 0;
	myoData[3] = 0;

	//Find pose efter setup er f�rdig
	if (finishedSetup) {
		getPose();
	}

	//Inds�t procent i data som skal sendes
	myoData[4] = (int)procent;

	//Print myoData i konsol
	if (finishedSetup && showMyoData)
		std::cout << "[" << myoData[0] << "," << myoData[1] << "," << myoData[2] << "," << myoData[3] << "," << myoData[4] << "," << myoData[5] << "," << myoData[6] << "," << myoData[7] << "]" << std::endl;
	else if (finishedSetup && showPose)	
		std::cout << "[" << myoData[0] << "," << myoData[1] << "," << myoData[2] << "," << myoData[3] << "," << myoData[4] << "]" << std::endl;

	counter++;

	//Visualiser EMG Data
	if(showVisualisation){
		//Counteren for emgData stiger
		dataHandler.UpdateAVG(filteredEmg);
		dataHandler.UpdateEMG(rawEmg);
		//Send data til TXT fil.(Python l�ser denne fil)
		if (counter % 100 == 1) { //1 gange i sekundet. 100/100 = 1
			dataHandler.SaveEMGData(rawEmg, "RawData.txt");
			dataHandler.SaveAVGData(filteredEmg, "FilteredData.txt");
		}
	}

}

//Funktionen kaldes n�r et nyt orientation er givet fra myoband
void DataCollector::onOrientationData(myo::Myo* myo, uint64_t timestap, const myo::Quaternion<float>& rotation) {

	int newRoll = 0;
	int newPitch = 0;
	int newYaw = 0;

	//Udregn euler angles(roll, pitch, yaw) udfra quaternion
	float rollRad = atan2(2.0f * (rotation.w() * rotation.x() + rotation.y() * rotation.z()),
		1.0f - 2.0f * (rotation.x() * rotation.x() + rotation.y() * rotation.y()));
	float pitchRad = asin(max(-1.0f, min(1.0f, 2.0f * (rotation.w() * rotation.y() - rotation.z() * rotation.x()))));
	float yawRad = atan2(2.0f * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
		1.0f - 2.0f * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));

	//Konverterer radianer til grader
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
			//sendOrientationToArduino();	
			//sendPoseToArduino();
			if (myoData[4] > 0)
			{
				sendGripperToArduino();
			}
			Sleep(10);
		}
		
	}
}
void DataCollector::sendGripperToArduino()
{
	byte Instruction = 0x10;
	byte params[2]{};
	int torque = myoData[4];
	params[1] = torque & 0xff;
	params[0] = (torque >> 8);
	Arduino.serialData(0x05, Instruction, params);
	Arduino.serialData(0x06, Instruction, params);
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

	int minmotor1{ 2300 }; //Ticks n�r motoren er i nul position
	int maxmotor1{ 1250 }; //Ticks n�r motoren er i maks position
	int zeromotor1 = minmotor1;
	//int zeromotor1 = (maxmotor1 - minmotor1) / 2 + minmotor1; //Ticks n�r motoren er midtvejs
	int fullmotor1Deg = 90; //Maks grader man kan bev�ge armen

	int minmotor2{ 2750 }; //Ticks n�r motoren er i nul position
	int maxmotor2{ 1350 }; //Ticks n�r motoren er i maks position
	int zeromotor2 = (maxmotor2 - minmotor2) / 2 + minmotor2; //Ticks n�r motoren er midtvejs
	int fullmotor2Deg = 100; //Maks grader man kan bev�ge armen

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
	t3.join();
}

//Kalibrerings funktion
void DataCollector::setupMyo(){
	//Loop indtil arduino er connected
	if (!Arduino.isConnected())
	{
		std::cout << "Arduino is not connected" << std::endl;
		Sleep(500);
	}

	std::cout << "Arduino is connected!" << std::endl;

	//Her kan brugeren bestemme hvilken data de vil vise i terminalen
	char input;

	std::cout << "Show raw data [y/n]" << std::endl; std::cin >> input;	
	if (input == 'y') showRawData = true;

	std::cout << "Show filtered data [y/n]" << std::endl; std::cin >> input;	
	if (input == 'y') showFilteredData = true;

	std::cout << "Show interval data [y/n]" << std::endl; std::cin >> input;	
	if (input == 'y') showIntervalData = true;

	if (!showIntervalData) {
		std::cout << "Show poses array [y/n]" << std::endl; std::cin >> input;
		if (input == 'y') showPoses = true;
	}
	std::cout << "Visualise raw data and filtered data [y/n]" << std::endl; std::cin >> input;
	if (input == 'y') showVisualisation = true;

	std::cout << "Starting setup" << std::endl;

	float temp = T;
	T = T_calibration; //S�t Tidskonstanten lig f�r kalibrering (Filteringen tager l�ngere tid, men spiker ikke s� meget)

	//K�rer igennem 4 poses og modtager max & min v�rdier for hver pod
	for(int i = 0; i < 4; i++){

		//vent p� brugeren trykker
		std::cout << "Perform pose " << i << std::endl;
		system("pause");

		//K�rer igennem alle filteret emg pods
		for(int j = 0; j < 8; j++){ //
			float value = filteredEmg[j];
			//Max og min er udregnet med to funktioner som skalerer v�rdien (potens & polynomium)
			MaxPods[i][j] = 7.7 * pow(value, -0.46) * value;
			MinPods[i][j] = (0.01 * pow(value, 2)) + (0.23 * value) - 1.42;
			if (MinPods[i][j] < 0) MinPods[i][j] = 0;
			//Print i terminalen
			std::cout << i << ": " << "Pod[" << j << "]: [" << (int)value << "] (" << MaxPods[i][j] << " " << MinPods[i][j] << ") " << std::endl;
		}
	
	}

	Sleep(500);
	
	//Kalibrere min fist
	std::cout << "Perform minimum fist" << std::endl;
	system("pause");
	//K�rer igennem alle filteret emg pods
	for (int i = 0; i < 8; i++) { //
		fistMin[i] = filteredEmg[i];
		std::cout << i << ": " << "Pod[" << i << "]: [" << fistMin[i] << "]" << std::endl;
	}

	Sleep(500);
	
	//Kalibrerer max fist
	std::cout << "Perform maximum fist" << std::endl;
	system("pause");
	//K�rer igennem alle filteret emg pods
	for (int i = 0; i < 8; i++) { //
		fistMax[i] = filteredEmg[i];
		std::cout << i << ": " << "Pod[" << i << "]: [" << fistMax[i] << "]" << std::endl;
	}

	Sleep(500);

	/// 
	/// ORIENTATION
	/// 

	std::cout << "Initialize Orientation Setup" << std::endl;
	Sleep(500);
	std::cout << "- Move arm to factory settings -" << std::endl;

	system("pause");
	std::cout << "Orientation calibrated" << std::endl;

	Sleep(500);

	std::cout << "Calibration Complete" << std::endl;
	system("pause");

	//Reset T v�rdien til hvad den var f�r kalibrering
	T = temp;
	//F�rdigg�r setup
	finishedSetup = true;

}
