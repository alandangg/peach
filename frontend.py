from tkinter import *
from setuptools import Command
from tkmacosx import Button
import os


def runPrune():
    os.system('python OpenCenterPruningScoreTest.py')


if __name__ == "__main__":
    
    root = Tk()
    root.title("Peach")

    
    button = Button(root, text="prune", bg = "red", fg="grey", command = runPrune)
    button.pack()

    

    root.mainloop()
