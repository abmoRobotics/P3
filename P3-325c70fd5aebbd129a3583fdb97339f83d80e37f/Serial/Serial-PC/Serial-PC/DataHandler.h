#pragma once
#include <iostream>
#include <fstream>
#include <string>

class DataHandler
{
public:
	void SaveData(int Data[100], std::string Filename);
	void Delete(std::string Filename);
	void Plot(std::string Filename);
	void Clear(std::string Filename);
	void Rename(char OldFilename[], char NewFilename[]);
private:
	std::ofstream DataFile;
};