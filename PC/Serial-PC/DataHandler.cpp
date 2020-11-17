#include "DataHandler.h"
#include <cstdio>
#include <iostream>
#include <fstream>

void DataHandler::UpdateEMG(int Data[8]){
    //Move all rows up by one
    for (int i = 0; i < 99; i++)
    {
        for (int k = 0; k < 8; k++)
        {
            EMG[i][k] = EMG[i + 1][k];
        }
    }

    //Update the buttom row from the new data
    for (int i = 0; i < 8; i++)
    {
        EMG[99][i] = Data[i];
    }
}

void DataHandler::UpdateAVG(float Data[8]){
    //Move all rows up by one
    for (int i = 0; i < 99; i++)
    {
        for (int k = 0; k < 8; k++)
        {
            AVG[i][k] = AVG[i + 1][k];
        }
    }

    //Update the buttom row from the new data
    for (int i = 0; i < 8; i++)
    {
        AVG[99][i] = Data[i];
    }
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

void DataHandler::SaveEMGData(int Data[8], std::string Filename)
{
    UpdateEMG(Data);
    DataFile.open(Filename);
    for (int i = 0; i < 100; i++)
    {
        for (int k = 0; k < 8; k++)
        {
            DataFile << EMG[i][k] << "\n";
        }
    }
    DataFile.close();
}

void DataHandler::SaveAVGData(float Data[8], std::string Filename)
{
    UpdateAVG(Data);
    DataFile.open(Filename);
    for (int i = 0; i < 100; i++)
    {
        for (int k = 0; k < 8; k++)
        {
            DataFile << AVG[i][k] << "\n";
        }
    }
    DataFile.close();
}