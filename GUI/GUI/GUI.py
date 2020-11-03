

import tkinter
from tkinter import *
#from tkinter import ttk

import numpy as np
import matplotlib.pyplot as plt



def main():
    print("Hello World!")
    data = readFile()
    Graph(data)
   #GUI()

def readFile():
    print("Reading text file")
    f = open("file.txt", "r")
    data = []
    #Read each line
    for x in f:
        data.append(x)
        print(x)
    print(len(data))
    f.close()
    return data

def Graph(data):
    plt.axis([1, 100, -10, 10])

    plt.plot(data)

    plt.ylabel('Voltage difference')
    plt.xlabel('')

    plt.show()

def GUI():
    window = Tk()

    window.title("Phone Assembly")
    window.geometry("750x800")
    window.configure(background="#bacbe8")
    window.mainloop()

if __name__ == "__main__":
    main()