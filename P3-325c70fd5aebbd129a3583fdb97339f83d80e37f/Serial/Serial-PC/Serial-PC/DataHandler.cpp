#include "DataHandler.h"
#include <cstdio>
#include <iostream>
#include <fstream>

void DataHandler::SaveData(int Data[100], std::string Filename)
{
    DataFile.open(Filename);
    for (int i = 0; i < 100; i++)
    {
        DataFile << Data[i] << "\n";
    }
    DataFile.close();
}

void DataHandler::Delete(std::string Filename)
{

}

void DataHandler::Plot(std::string Filename)
{
}

void DataHandler::Clear(std::string Filename)
{
    DataFile.open(Filename, std::ofstream::out | std::ofstream::trunc);
    DataFile.close();
}

void DataHandler::Rename(char OldFilename[], char NewFilename[])
{
    if (rename(OldFilename, NewFilename) != 0)
        perror("Error renaming file");
    else
        std::cout << "File renamed successfully";
}
