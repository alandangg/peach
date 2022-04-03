from tkinter import *
from tkinter import filedialog
import os


def runPrune():
    os.system('python OpenCenterPruningScoreTest.py')

def initUI():

    #Top bar
    root. title("Peach")
    root.iconbitmap('frontend/peach.ico')
    root.iconphoto(False, PhotoImage(file='frontend/peach.png'))


    #Main GUI
    button = Button(root, text="prune", fg="black", highlightbackground="red", highlightthickness= 0,
                    width= 20,command = runPrune)
    button.grid(column= 0, row= 1)

    entry = Entry(root, width=25, borderwidth=10)
    entry.insert(0, "Entry")
    entry.grid(column=0, row= 0)

    #File Selection
    root.pcdSelect = filedialog.askopenfilename(title="Select a point cloud...", filetypes=(("all files", "*.*")))
    

if __name__ == "__main__":
    
    root = Tk()
    initUI()
   
    

    

    root.mainloop()
