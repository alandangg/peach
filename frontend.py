from asyncio import selector_events
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QLabel, QSlider, QComboBox, QFileDialog
from PyQt5 import uic
import sys
import os
import OpenCenterPruningScoreTest as oc



class UI(QMainWindow):


    def __init__(self):
        super().__init__()

        uic.loadUi("frontend/peach.ui", self)

        self.pcdbutton = self.findChild(QPushButton, "pcd_button")
        self.cylinderbutton = self.findChild(QPushButton, "cylinder_button")
        self.databutton = self.findChild(QPushButton, "treedata_button")
        self.prunebutton = self.findChild(QPushButton, "prune_button")
        self.pcdlabel = self.findChild(QLabel, "pcd_label")
        self.prunemethod = self.findChild(QComboBox, "prunemethods_combobox")
        self.pruneslider = self.findChild(QSlider, "prune_slider")
        

        
        self.pcdbutton.clicked.connect(self.pcdClicker)
       
        self.cylinderbutton.clicked.connect(self.cylinderClicker)
        self.databutton.clicked.connect(self.dataClicker)
        self.prunebutton.clicked.connect(self.pruneClicker)
        self.prunemethod.currentIndexChanged.connect(self.prunemethodSelect)
        self.pruneslider.valueChanged.connect(self.silderGetter)


        self.show()

    def prunemethodSelect(self):
        pruneSelect = self.prunemethod.currentText()
        #print(pruneSelect)
        return pruneSelect
    
    def silderGetter(self):
        slider = self.pruneslider.value()
        #print(slider)
        return slider

    
   
    def pcdClicker(self):
        
        # Open File Dialog
        fpcd = QFileDialog.getOpenFileName(self, "Open Point Cloud Data", "", "All Files (*)")
        if fpcd:
            file1 = open("pcdPath.txt", "w") 
            file1.write(str(fpcd[0]))
            file1.close
            self.pcdbutton.setText(str(fpcd[0]))
            

    
            
      
            
           
    def cylinderClicker(self):
        # Open File Dialog
        
        fcylinder = QFileDialog.getOpenFileName(self, "Open Cylinder Data", "", "All Files (*)")
        if fcylinder:
            file1 = open("cylinderPath.txt", "w")
            file1.write(str(fcylinder[0]))
            file1.close
            self.cylinderbutton.setText(str(fcylinder[0]))
            
          
            
        
            
    def dataClicker(self):
        # Open File Dialog
        
        fdata = QFileDialog.getOpenFileName(self, "Open Tree Data", "", "All Files (*)")
        if fdata:
            file1 = open("dataPath.txt", "w")
            file1.write(str(fdata[0]))
            file1.close
            self.databutton.setText(str(fdata[0]))
           
        
           

    def pruneClicker(self):
       
        oc.runner(self)
        


app = QApplication(sys.argv)
UIWindow = UI()
app.exec_()