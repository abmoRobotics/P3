#pragma once
#include <iostream>
#include <fstream>
#include <string>

class DataHandler //Klasse, hvori metoder til håndtering af data ligger(Gemme, slette filer mm.)
{
public:
	void SaveEMGData(int Data[100][8], std::string Filename);	//Gem EMG data i en fil.
	void Delete(std::string Filename);						//Slet en fil
	void Plot(std::string Filename);						//Plot Data(Ikke implementeret)
	void Clear(std::string Filename);						//Slet indhold i fil
	void Rename(char OldFilename[], char NewFilename[]);	//Omdøb filnavn
	void SaveData(int Data[8], std::string Filename);		//Gem Data[8] i en fil
private:
	std::ofstream DataFile;									//Objekt, hvor metoder til håndtering af filer ligger i.
};