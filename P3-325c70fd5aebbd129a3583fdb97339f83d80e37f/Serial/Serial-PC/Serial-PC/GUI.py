import tkinter
from tkinter import *
#from tkinter import ttk

import numpy as np
import matplotlib.pyplot as plt


def main():
    print("Hello World!")
    Graph()
    #GUI()


def Graph():
    plt.axis([0, 10, 0, 1])

    for i in range(10):
        y = np.random.random()
        plt.scatter(i, y)
        plt.pause(0.05)

    plt.show()

def GUI():
    window = Tk()

    window.title("Phone Assembly")
    window.geometry("750x800")
    window.configure(background="#bacbe8")
    window.mainloop()

if __name__ == "__main__":
    main()