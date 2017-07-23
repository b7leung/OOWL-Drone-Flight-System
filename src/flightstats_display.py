#!/usr/bin/env python

import rospy
import sys
import collections

#from flightstats_receiver import FlightstatsReceiver

from PySide import QtGui, QtCore

# an extension of FlightstatsReciever that also provides a GUI interface
#class FlightstatsDisplay(FlightstatsReceiver, QtGui.QWidget):
class FlightstatsDisplay(QtGui.QWidget):

    def __init__(self):
        
        super(FlightstatsDisplay,self).__init__()
        # Key is the variable name; tuple value is in format 
        # (description, value, units)
        self.defaultValue = "?"
        self.flightInfo = collections.OrderedDict()
        self.flightInfo["batteryPercent"]=("Battery Left: ", self.defaultValue, "%")
        self.flightInfo["state"]=("Status: ", self.defaultValue, "")
        self.flightInfo["altitude"]=("Altitude: ", self.defaultValue, "ft")
        self.flightInfo["rotX"]=("Left/Right Tilt: ", self.defaultValue, u'\N{DEGREE SIGN}')
        self.flightInfo["rotY"]=("Front/Back Tilt: ", self.defaultValue, u'\N{DEGREE SIGN}')
        self.flightInfo["rotZ"]=("Rotation Amount: ", self.defaultValue, u'\N{DEGREE SIGN}')
        self.flightInfo["velX"]=("Left/Right Velocity: ", self.defaultValue, "mm/s")
        self.flightInfo["velY"]=("Front/Back Velocity: ", self.defaultValue, "mm/s")
        self.flightInfo["velZ"]=("Up/Down Velocity: ", self.defaultValue, "mm/s")


        self.setGeometry(300,300,450,450)
        self.setWindowTitle("AR Drone Live Flight Statistics")

        self.grid = QtGui.QGridLayout()
        self.UpdateGridInfo()

        self.show()

    def UpdateGridInfo(self):
        row = 0
        for key, value in self.flightInfo.iteritems():
            # delete existing item if it exists
            self.DeleteGridWidget(row, 0)
            self.DeleteGridWidget(row, 1)
            # add current data
            self.grid.addWidget(QtGui.QLabel(value[0]), row,0, QtCore.Qt.AlignCenter)
            self.grid.addWidget(QtGui.QLabel(str(value[1])+" " + value[2]), row, 1, QtCore.Qt.AlignCenter)
            row +=1
            line = QtGui.QFrame()
            line.setFrameShape(QtGui.QFrame.Shape.HLine)
            self.grid.addWidget(line, row, 0, 1, 2)
            row +=1
        self.setLayout(self.grid)


    def DeleteGridWidget(self, row, col):
        item = self.grid.itemAtPosition(row,col)
        if item is not None:
            widget = item.widget()
            if widget is not None:
                self.grid.removeWidget(widget)
                widget.deleteLater()
        

    def ProcessNavdata(self):
        self.UpdateGridInfo()


if __name__=="__main__":
    rospy.init_node('FlightstatsDisplay')
    app = QtGui.QApplication(sys.argv)
    display=FlightstatsDisplay()
    sys.exit(app.exec_())
