#include "DataCollectorFilter.h"
#include <math.h>
#include <atlstr.h>
#include <string.h>


DataCollector::DataCollector()
{

}
//Funktionen kaldes når vi modtager et nyt pose fra myo
void DataCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {


}

//Funktionen kører hele tiden og tjekker om man dobbeltapper (Funktionen bliver kaldt i main og startes i et nyt multithread)
void DataCollector::fistModeTimer() {
	while (true) {
		Sleep(10);
		if (fistControlOn) { //Når man har lavet bevægelsen for at lukke gripper
			allowFist = false; //Opdater så gripperen ikke kan åbnes imens den lukkes
			PlaySound(TEXT("swoosh.wav"), NULL, SND_SYNC);
			//Sleep(1000);
			allowRead = true; //Tillader at procenten opdateres
			Sleep(3000);
			fistControlOn = false;
			std::cout << "LOCKED" << std::endl;
			PlaySound(TEXT("targetLocked.wav"), NULL, SND_SYNC);
			allowRead = false; //Gør at procenten ikke længere opdateres
			//Sleep(1000);
			
			allowFist = true; //Opdater, så man igen kan åbne gripperen. (Gøres for at sikre gripperen ikke åbnes ved fejl)
		}
		
	}
}
// Får rawEmg data og sætter det ind i et array: emgSamples
void DataCollector::getData(const int8_t* emg) {
	//Modtag data
	for (int i = 0; i < 8; i++)
	{
		previousRawEmg[i] = rawEmg[i]; //Gem tidligere emg data
		rawEmg[i] = emg[i]; //Gem nuværende emg data
	}

	//Printer data i terminalen
	if (finishedSetup && showRawData) {
		for (int i = 0; i < 8; i++) {
			std::cout << "[" << rawEmg[i] << "]";
		}
		std::cout << " " << std::endl;
	}

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


	//Print filtreret data
	if (showFilteredData && finishedSetup) {
		for (int i = 0; i < 8; i++) {
			std::cout << "[" << (int)filteredEmg[i] << "]";
		}
		std::cout << " " << std::endl;
	}

}

//Sætter de første 4 Myodata, afhængig af pose
void DataCollector::getPose() {

	//Tjekker hvilken pose der bliver lavet
	for (size_t i = 0; i < 6; i++)
	{
		movements[i] = 1;
	}
	for (int i = 0; i < 6; i++) { // Kører igennem alle movements
		for (int j = 0; j < 8; j++) { // Kører igennem alle filteret pods
			if (filteredEmg[j] < MinPods[i][j] || filteredEmg[j] > MaxPods[i][j]) { //Hvis pod IKKE er i invervallet
				movements[i] = false;
			}
		}
	}

	//Fixer så flere poses ikke kan være aktive på samme tid
	//Kører i gennem alle poses, tjekker om de er aktive, og deaktiverer resten
	for (int i = 0; i < 6; i++) {
		if (movements[i]) {
			for (int j = 0; j < 5; j++) { //Reset alle movements til 0 //Hvorfor 5 når der er 6 poses?
				movements[j] = false;
			}
			movements[i] = true; //Sæt den aktive pose til true igen
		}
	}

	if (movements[4] == true && !fistControlOn && allowFist) {

		//Begynd fistmode hvis procent er 0
		if (procent == 0) {
			fistControlOn = true;

		}
	}

	if (movements[5]) {
		releaseCounter++;
	}
	else
	{
		releaseCounter = 0;
	}

	if (movements[5] == true && releaseCounter >= 100 && !fistControlOn && allowFist) {
		procent = 0;
		myoData[4] = (int)procent;
		//std::cout << "RELEASE MEEEE RELEASE MY BODY\n";
		releaseGripper = true;
		
	}

	//Fist
	if (fistControlOn) {
		//Sætter alle "movements" til at være 0	 
		for (int i = 0; i < 6; i++) {
			movements[i] = 0;
		}


		//Regner summen af alle pods ud
		int sum = 0;
		int sumMin = 0;
		int sumMax = 0;
		for (int i = 0; i < 8; i++) {
			sum = filteredEmg[i] + sum; //Sum af nuværende pods
			sumMin = fistMin[i] + sumMin; //Sum af kalibreret min fist
			sumMax = fistMax[i] + sumMax; //Sum af den kalibreret max Fist
		} 

		//Regner procent
		procent = (((float)sum - (float)sumMin) / ((float)sumMax - (float)sumMin)) * 100;
		//I tilfælde at man kommer over 100%
		if (procent > 100) {
			procent = 100;
		}
		//i tilfælde at man kommer under 0%
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
		for (int i = 0; i < 6; i++) {
			std::cout << " [" << movements[i] << "]";
		}
		std::cout << " [" << myoData[4] << "%]" << std::endl;
	}

}

//Funktionen kaldes når vi modtager nyt emgData (hver 5 millisekund eller noget fast)
void DataCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
	myo->unlock(myo::Myo::unlockHold); //myo bandet unlockes hver 5 millisekund for at sikre at dobbeltap virker på første forsøg) //Er dette stadig relevant?

	//Modtag rå emgData og indsæt i en matrice, samt flyt tidligere data i en anden matrice
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
	if(allowRead)
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
		//Send data til TXT fil.(Python læser denne fil)
		if (counter % 100 == 1) { //1 gange i sekundet. 100/100 = 1
			dataHandler.SaveEMGData(rawEmg, "RawData.txt");
			dataHandler.SaveAVGData(filteredEmg, "FilteredData.txt");
		}
	}

}

//Funktionen kaldes når et nyt orientation er givet fra myoband
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

	if (finishedSetup && newRoll == 100000) {
		std::cout << newRoll << "," << newPitch << "," << newYaw << std::endl;
	}

}

//Sender data til arduino hele tiden 
void DataCollector::arduinoThread() {
	while(1) {
		if (finishedSetup)
		{
			std::chrono::steady_clock sc;   // create an object of `steady_clock` class
			auto start = sc.now();     // start timer

			
			if (fistControlOn || releaseGripper)
			{
				sendGripperToArduino();
				Arduino.setJointVelocity(3, 0, 0x01);
				Arduino.setJointVelocity(4, 0, 0x01);

			}
			else
			{
				sendVelocityToArduino();
				sendPoseToArduino();
			}
			//Arduino.setJointPosition(1000, 1);
			auto end = sc.now();       // end timer (starting & ending is done by measuring the time at the moment the process started & ended respectively)
			auto time_span = static_cast<std::chrono::duration<double>>(end - start);   // measure time span between start & end
			//std::cout << "Operation took: " << time_span.count() << " seconds !!!\n";
		}
		
	}
}
void DataCollector::sendGripperToArduino()
{
	int torque = myoData[4];
	if (fistControlOn)
	{
		//std::cout << torque << std::endl;
		Arduino.setGripperTorque(torque, 0x05, 0x01);
		Arduino.setGripperTorque(torque, 0x06, 0x01);
	}
	else if (releaseGripper)
	{
		Arduino.setGripperTorque(torque, 0x05, 0x02);
		releaseGripper = false;
	}
	
}

//Sender pose til arduino (up, down, out, in) //SKAL LAVES OM
void DataCollector::sendPoseToArduino()
{
	if (finishedSetup)
	{
		int pitch{ myoData[6] };
		int roll{ myoData[5] };
		
		if (abs(pitch) > 0 && abs(pitch) < 91)
		{
			Arduino.setJointPosition(1, abs(pitch));
		}
			
		
		

		if (roll < 66 && roll > -66)
		{
			Arduino.setJointPosition(2, roll+65);
		}
		//std::cout << pitch << "   " << roll << std::endl;
		
	}
	
}


//Sender orientationen til arduino //SKAL LAVES OM
void DataCollector::sendVelocityToArduino()
{
	int velocity{ 15 };
	if (movements[1])
	{
		Arduino.setJointVelocity(3, velocity, 0x01);
	}
	else if (movements[0])
	{
		Arduino.setJointVelocity(3, velocity, 0x02);
	}
	else if (movements[2])
	{
		Arduino.setJointVelocity(4, velocity, 0x01);
	}
	else if (movements[3])
	{
		Arduino.setJointVelocity(4, velocity, 0x02);
	}
	else
	{
		Arduino.setJointVelocity(4, velocity, 0x03);
	}
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
	if(Arduino.isConnected()) std::cout << "Arduino is connected!" << std::endl;

	

	//Her kan brugeren bestemme hvilken data de vil vise i terminalen
	char input;
	//showMyoData = true;
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
	T = T_calibration; //Sæt Tidskonstanten lig før kalibrering (Filteringen tager længere tid, men spiker ikke så meget)

	//Kører igennem 4 poses og modtager max & min værdier for hver pod
	for(int i = 0; i < 6; i++){

		//vent på brugeren trykker
		std::cout << "Perform pose " << i << std::endl;
		system("pause");

		//Kører igennem alle filteret emg pods
		for(int j = 0; j < 8; j++){ //
			float value = filteredEmg[j];
			//Max og min er udregnet med to funktioner som skalerer værdien (potens & polynomium)
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
	//Kører igennem alle filteret emg pods
	for (int i = 0; i < 8; i++) { //
		fistMin[i] = filteredEmg[i];
		std::cout << i << ": " << "Pod[" << i << "]: [" << fistMin[i] << "]" << std::endl;
	}

	Sleep(500);
	
	//Kalibrerer max fist
	std::cout << "Perform maximum fist" << std::endl;
	system("pause");
	//Kører igennem alle filteret emg pods
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


	startRoll = roll;
	//std::cout << "roll: " << startRoll << std::endl;

	startPitch = pitch;
	//std::cout << "Pitch: " << startPitch << std::endl;

	startYaw = yaw;
	//std::cout << "Yaw: " << startYaw << std::endl;

	/// 
	/// COMPLETE
	/// 
	Sleep(500);

	std::cout << "Calibration Complete" << std::endl;
	system("pause");

	//Reset T værdien til hvad den var før kalibrering
	T = temp;
	//Færdiggør setup
	finishedSetup = true;
	//std::terminate();
}


