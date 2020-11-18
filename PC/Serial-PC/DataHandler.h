#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <iostream>

class DataHandler //Klasse, hvori metoder til håndtering af data ligger(Gemme, slette filer mm.)
{
public:
	void Delete(std::string Filename);						//Slet en fil
	void Plot(int Data[16]);									//Plot Data(Ikke implementeret)
	void Clear(std::string Filename);						//Slet indhold i fil
	void Rename(char OldFilename[], char NewFilename[]);	//Omdøb filnavn
	void SaveEMGData(int Data[8], std::string Filename);		//Gem Data[8] i en fil
	void SaveAVGData(float Data[8], std::string Filename);		//Gem Data[8] i en fil
	void UpdateEMG(int Data[8]);	//Gem EMG data i en fil.
	void UpdateAVG(float Data[8]);	//Gem EMG data i en fil.

private:
	std::ofstream DataFile;			//Objekt, hvor metoder til håndtering af filer ligger i.
	float EMG[100][8] = { 0 };
	float AVG[100][8] = { 0 };

};