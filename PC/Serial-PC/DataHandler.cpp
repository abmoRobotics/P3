#include "DataHandler.h"
#include <cstdio>
#include <iostream>
#include <fstream>

void DataHandler::SaveEMGData(int Data[100][8], std::string Filename)
{
    DataFile.open(Filename);
    for (int k = 0; k < 8; k++)
    {
        for (int i = 0; i < 100; i++)
        {
            DataFile << Data[i][k] << "\n";
        }
    }

    DataFile.close();
}

void DataHandler::Delete(std::string Filename)
{

}

void DataHandler::Plot(int Data[16])
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

void DataHandler::SaveData(int Data[8], std::string Filename)
{
    DataFile.open(Filename);
    for (int k = 0; k < 8; k++)
    {
        DataFile << Data[k] << "\n";
    }

    DataFile.close();
}