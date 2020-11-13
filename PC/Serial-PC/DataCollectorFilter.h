#pragma once
#include <myo/myo.hpp>
#include <thread>
#include "Control.h"
#include "DataHandler.h"

//Sound include
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

class DataCollector : public myo::DeviceListener {
private:
	//Thresholds for hver bevælgese (default værdier)
	int upThreshold = 92;
	int downThreshold = 87;
	int outThreshold = 30;
	int inThreshold = 26;
	int fistMaxThreshold = 40;
	int fistMinThreshold = 215;

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

	float T = 0.17; //Tidskonstant for filtreringen (std. værdi = 1.7)
	float T_calibration = 0.6; //Tidskonstanten under kalibrering
	float Ts = 0.005; //Samplingstid for systemet. Bruges i filtereringen. (200Hz, derfor 5 ms)

	float filteredEmg[8] = {0, 0, 0, 0, 0, 0, 0, 0 }; //Filtering af hver pods emg Data u(t)
	float previousFilteredEmg[8] = {0, 0, 0, 0, 0, 0, 0, 0 }; //Filtering af hver pods emg Data u(t-dt)

	int rawEmg[8]; //raw emg data E(t)
	int previousRawEmg[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Previous emg data, starts empty E(t-dt)

	int counter = 0; //counter af antal emgData

	int sampleSize = 100; //200 = 1 sekund, 100 = 0.5 sekund osv.

	int myoData[8]; //myoData som bliver sendt videre til arduino
	int MaxPods[4][8];
	int MinPods[4][8];
	int fistMin[8];
	int fistMax[8];

	//Debug variabler
	bool showRawData = false;
	bool showFilteredData = false;
	bool showIntervalData = false;
	bool showPoses = false;
	bool showMyoData = false;
	bool showPose = false;
	bool showOrientation = false;

	arduinoCOM Arduino;
	DataHandler dataHandler;

	void fistModeTimer();
	void onPose(myo::Myo*, uint64_t, myo::Pose);
	void onEmgData(myo::Myo*, uint64_t, const int8_t*);
	void onOrientationData(myo::Myo*, uint64_t, const myo::Quaternion<float>&);
	void setupMyo();
	void setupMyo2();
	int GETPOSE();
	void getData(const int8_t*);
	void applyFilter();
	void getPose();
	void sendOrientationToArduino();
	void sendPoseToArduino();
	void arduinoThread();
public:
	void startThreads();
};