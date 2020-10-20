#pragma once
#include <myo/myo.hpp>
#include <thread>
#include "Control.h"

//Sound include
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

class DataCollector : public myo::DeviceListener {
private:
	//Thresholds for hver bevælgese (default værdier)
	int upThreshold = 100;
	int downThreshold = 120;
	int outThreshold = 30;
	int inThreshold = 50;
	int fistMaxThreshold = 40;
	int fistMinThreshold = 270;

	//Orientations
	int roll = 0;
	int pitch = 0;
	int yaw = 0;

	//Start orientations to calibrate
	int startRoll = 0;
	int startPitch = 0;
	int startYaw = 0;

	//Er true når emgData array er fyldt
	bool avgReady = false;
	bool fistModeOn = false;
	bool finishedSetup = false;

	float procent = 0; //Procent af fist

	int average[8]; //Gennemsnit af hver pod. Ex. average[0] viser gennemsnittet af pod 0 over x sekund.

	int emgSamples[8]; //raw emg data

	int counter = 0; //counter af antal emgData

	int emgData[100][8]; //Matrixe af emgData til hver pod over x sekund.

	int sampleSize = 100; //200 = 1 sekund, 100 = 0.5 sekund osv.

	int myoData[8]; //myoData som bliver sendt videre til arduino

	//Gennemsnit af hver bevægelse
	int fistAvg = 0;
	int upAvg = 0;
	int downAvg = 0;
	int outAvg = 0;
	int inAvg = 0;

	//Debug variabler
	bool showRawEmg = false;
	bool showMyoData = false;
	bool showPose = false;
	bool showOrientation = false;

	arduinoCOM Arduino;

	void fistModeTimer();
	void onPose(myo::Myo*, uint64_t, myo::Pose);
	void onEmgData(myo::Myo*, uint64_t, const int8_t*);
	void onOrientationData(myo::Myo*, uint64_t, const myo::Quaternion<float>&);
	void setupMyo();

	void GetData(const int8_t*);
	void CalculateAverage();
	void GetPose();
	void sendOrientationToArduino();
	void ArduinoThread();
public:
	void startThreads();
};