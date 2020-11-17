

#GUI Import
import tkinter
from tkinter import *

#General imports
import numpy as np
import array as arr
import os

#MatplotLib import (Plot grafer)
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib import animation



#Sti til txt filerne
PATH = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, os.pardir,'PC','Serial-PC'))


#Læs indholdet af en txt fil og returnere indholdet i et array
def readFile(FileName):
    print("Reading text file")
    #Åben filen ud fra FileName værdien
    f = open(PATH + '\\' + FileName, "r")
    #Deklarere data array som bliver fyldt op med indholdet 
    data = []
    #Læs hver linje i txt filen
    for x in f:
        data.append(float(x)) #Tilføj hver linje til arrayet
    f.close()
    return data


def Graphs():

    #Setup plot
    plt.ion()
    fig = plt.figure()

    #Smart solution to create 8 figures (Undgå hardcoding)
    ax = [] #Array af hver plot
    plotLocation =  [241, 242, 243, 244, 245, 246, 247, 248] #[Række, søjle, ID]

    #Automatisk lav 8 plots ved hjælp af while loop
    i = 0
    while i < 8:
        ax.append(fig.add_subplot(plotLocation[i])) #Lav ny plot
        ax[i].set_title(i) #Sæt tilten af ny plot
        ax[i].axis([0, 100, -100, 100]) #Definere akserne, [x_min, x_max, y_min, y_max]
        i += 1

    #Deklarere hver linje. Linje0 = Rå data, Linje01 = Gennemsnits data osv..
    x = np.linspace(0, 100, 100)
    #Each line hardcoded cuz fuck it
    line0, = ax[0].plot(x, np.zeros((100)), 'r-') #Rå EMG data af pod 0, rød linje
    line01, = ax[0].plot(x, np.zeros((100)), 'b-') #Gennemsnit EMG data af pod 0, blå linje
    line1, = ax[1].plot(x, np.zeros((100)), 'r-') # 
    line11, = ax[1].plot(x, np.zeros((100)), 'b-') #
    line2, = ax[2].plot(x, np.zeros((100)), 'r-') # 
    line21, = ax[2].plot(x, np.zeros((100)), 'b-') # 
    line3, = ax[3].plot(x, np.zeros((100)), 'r-') #
    line31, = ax[3].plot(x, np.zeros((100)), 'b-') #
    line4, = ax[4].plot(x, np.zeros((100)), 'r-') #
    line41, = ax[4].plot(x, np.zeros((100)), 'b-') #
    line5, = ax[5].plot(x, np.zeros((100)), 'r-') #
    line51, = ax[5].plot(x, np.zeros((100)), 'b-') # 
    line6, = ax[6].plot(x, np.zeros((100)), 'r-') #
    line61, = ax[6].plot(x, np.zeros((100)), 'b-') #
    line7, = ax[7].plot(x, np.zeros((100)), 'r-') # 
    line71, = ax[7].plot(x, np.zeros((100)), 'b-') # 

    while 1: #While loop som kører for evigt

        #Modtag data fra txt filerne
        rawData = readFile("RawData.txt")
        avgData = readFile("FilteredData.txt")
        data = ConvertArray(rawData)
        dataavg = ConvertArray(rawData)

        #Opdatere hver plot med det nye data
        #:, betyder søjle.
        line0.set_ydata(data[:,0]) #Opdatere linje0's data med pod 0's data. (Søjlen i data[0])
        line01.set_ydata(dataavg[:,0])
        line1.set_ydata(data[:,1])
        line11.set_ydata(dataavg[:,0])
        line2.set_ydata(data[:,2])
        line21.set_ydata(dataavg[:,0])
        line3.set_ydata(data[:,3])
        line31.set_ydata(dataavg[:,0])
        line4.set_ydata(data[:,4])
        line41.set_ydata(dataavg[:,0])
        line5.set_ydata(data[:,5])
        line51.set_ydata(dataavg[:,0])
        line6.set_ydata(data[:,6])
        line61.set_ydata(dataavg[:,0])
        line7.set_ydata(data[:,7])
        line71.set_ydata(dataavg[:,0])

        fig.canvas.draw()
        fig.canvas.flush_events()

#Starter en GUI (bliver ikke brugt lige nu, men matplotlib's figurer kan arbejde sammen med tkinter's GUI)
def GUI():
    window = Tk()

    window.title("Title")
    window.geometry("750x800")
    window.configure(background="#bacbe8")
    window.mainloop()

#Konverterer en 800x1 array til et 100x8 array
def ConvertArray(Input):
    #Deklarere et tomt array
    Data = np.zeros((100,8));
    i = 0
    index = 0
    #for loop magi
    for x in Input:
        Data[i % 100][index] = Input[i]
        if i % 100 == 99:
            index += 1
        i += 1
    return Data

#Main funktion
def main():
    Graphs()

if __name__ == "__main__":
    main()