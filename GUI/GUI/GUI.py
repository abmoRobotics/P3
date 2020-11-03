

import tkinter
from tkinter import *
#from tkinter import ttk

import numpy as np
import array as arr
import matplotlib.pyplot as plt
import os
from matplotlib.pyplot import figure
from matplotlib import animation

import time


PATH = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, os.pardir,'P3-325c70fd5aebbd129a3583fdb97339f83d80e37f','Serial','Serial-PC','Serial-PC'))


def main():
    Graphs()

def readFile(FileName):
    print("Reading text file")
    f = open(PATH + '\\' + FileName, "r")
    data = []
    #Read each line
    for x in f:
        data.append(int(x))
    f.close()
    return data


def Graphs():

    plt.ion()

    fig = plt.figure()

    #Smart solution to create 8 figures (Undgå hardcoding)
    ax = []
    plotLocation =  [241, 242, 243, 244, 245, 246, 247, 248]
    i = 0
    while i < 8:
        ax.append(fig.add_subplot(plotLocation[i]))
        ax[i].set_title(i)
        ax[i].axis([0, 100, -100, 100])
        i += 1

    x = np.linspace(0, 100, 100)
    #Each line hardcoded cuz fuck it
    line0, = ax[0].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line01, = ax[0].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma
    line1, = ax[1].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line11, = ax[1].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma
    line2, = ax[2].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line21, = ax[2].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma
    line3, = ax[3].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line31, = ax[3].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma
    line4, = ax[4].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line41, = ax[4].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma
    line5, = ax[5].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line51, = ax[5].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma
    line6, = ax[6].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line61, = ax[6].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma
    line7, = ax[7].plot(x, np.zeros((100)), 'r-') # Returns a tuple of line objects, thus the comma
    line71, = ax[7].plot(x, np.zeros((100)), 'b-') # Returns a tuple of line objects, thus the comma

    while 1:
        rawData = readFile("EMGdata.txt")
        avgData = readFile("AVGdata.txt")
        data = ConvertArray(rawData)

        line0.set_ydata(data[:,0])
        line01.set_ydata(avgData[0])
        line1.set_ydata(data[:,1])
        line11.set_ydata(avgData[1])
        line2.set_ydata(data[:,2])
        line21.set_ydata(avgData[2])
        line3.set_ydata(data[:,3])
        line31.set_ydata(avgData[3])
        line4.set_ydata(data[:,4])
        line41.set_ydata(avgData[4])
        line5.set_ydata(data[:,5])
        line51.set_ydata(avgData[5])
        line6.set_ydata(data[:,6])
        line61.set_ydata(avgData[6])
        line7.set_ydata(data[:,7])
        line71.set_ydata(avgData[7])

        fig.canvas.draw()
        fig.canvas.flush_events()

def GUI():
    window = Tk()

    window.title("Phone Assembly")
    window.geometry("750x800")
    window.configure(background="#bacbe8")
    window.mainloop()

def ConvertArray(Input):
    Data = np.zeros((100,8));
    i = 0
    index = 0
    for x in Input:
        Data[i % 100][index] = Input[i]
        if i % 100 == 99:
            index += 1
        i += 1
    return Data


if __name__ == "__main__":
    main()