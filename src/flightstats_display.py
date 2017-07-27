#!/usr/bin/env python

import rospy
import sys
import collections
import copy
import functools

from flightstats_receiver import FlightstatsReceiver
from drone_status import DroneStatus

from PySide import QtGui, QtCore

# toggles on how many decimal places to view
FULL_VIEW = 2
MEDIUM_VIEW = 1
SIMPLE_VIEW = 0

# frequency to update GUI display, in ms
UPDATE_FREQUENCY = 10
REAL_TIME_FREQ = 5

curr_view = None
curr_units_system = None
curr_refresh_rate = None

# A Widget that provides all the navadata recieved as a flightstatsreceiver
# in a easy to view grid layout 
class MainGridWidget(FlightstatsReceiver, QtGui.QWidget):

    def __init__(self):
        super(MainGridWidget,self).__init__()

        # setting up grid
        self.grid = QtGui.QGridLayout()
        self.UpdateGridInfo()
        self.setLayout(self.grid)

        # Start timer that will refresh the widget every specified number of ms
        self.updateTimer = QtCore.QTimer(self)
        self.updateTimer.timeout.connect(self.UpdateGridInfo)
        self.updateTimer.start(UPDATE_FREQUENCY)
    

    # Updates GUI with the latest information in the Dictionary
    def UpdateGridInfo(self):

        # process dictionary values first so it's easy to read
        processedDict = self.ProcessValues()
        
        # update grid with processed values
        row = 0
        for key, value in processedDict.iteritems():
            # delete existing item if it exists
            self.DeleteGridWidget(row, 0)
            self.DeleteGridWidget(row, 1)
            # add current data
            self.grid.addWidget(QtGui.QLabel(value[0]), row,0, QtCore.Qt.AlignCenter)
            self.grid.addWidget(QtGui.QLabel(str(value[1])+ " " + value[2] + " " +  value[3]),row, 1, QtCore.Qt.AlignCenter)
            row +=1
            line = QtGui.QFrame()
            line.setFrameShape(QtGui.QFrame.Shape.HLine)
            self.grid.addWidget(line, row, 0, 1, 2)
            row +=1
        self.setLayout(self.grid)


    # Performs unit conversions, rounding, etc so that raw data will be easy to read
    # Returns the processed dictionary
    def ProcessValues(self):
        
        dict = copy.deepcopy(self.flightInfo)

        # Round battery % to whole number
        if (dict["batteryPercent"])[1] != self.defaultValue:
            (dict["batteryPercent"])[1]= int( (dict["batteryPercent"])[1] )
        
        # Convert drone status from numbers to words
        StatusMessages = {
            DroneStatus.Emergency : 'Emergency',
            DroneStatus.Inited    : 'Initialized',
            DroneStatus.Landed    : 'Landed',
            DroneStatus.Flying    : 'Flying',
            DroneStatus.Hovering  : 'Hovering',
            DroneStatus.Test      : 'Test',
            DroneStatus.TakingOff : 'Taking Off',
            DroneStatus.GotoHover : 'Going to Hover Mode',
            DroneStatus.Landing   : 'Landing',
            DroneStatus.Looping   : 'Looping'
            }
        if (dict["state"])[1] != self.defaultValue:
            (dict["state"])[1]= StatusMessages[(dict["state"])[1]]

        # Convert Altitude from mm to m
        if (dict["altitude"])[1] != self.defaultValue:
            pass
            # converting from mm to m
            #(dict["altitude"])[1]= (dict["altitude"])[1] / 1000.0
            # converting from M to ft
            #(dict["altitude"])[1]= (dict["altitude"])[1] / 0.3048
        
        # Convert velocity from mm/s to in/s
        for vel in ["velX", "velY", "velZ"]:
            if (dict[vel])[1] != self.defaultValue:
                # converting from mm/s to in/s
                (dict[vel])[1]= (dict[vel])[1] * 0.0393701

        # rounding all values to make it easier to view
        for num in ["altitude", "rotX", "rotY", "rotZ", "velX", "velY", "velZ"]:
            if (dict[num])[1] != self.defaultValue:
                if curr_view == SIMPLE_VIEW:
                    # rounds to whole numbers
                    (dict[num])[1]= int((dict[num])[1])
                elif curr_view == MEDIUM_VIEW: 
                    # rounds to hundredths places
                    (dict[num])[1]= int((dict[num])[1] * 10) / 10.0
                elif curr_view == FULL_VIEW:
                    fullround = 10000000.0
                    (dict[num])[1]= int((dict[num])[1] * fullround) / fullround
        
        # Add corresponding direction strings
        for direction in ["rotX", "velX"]:
            if (dict[direction])[1] != self.defaultValue:
                if (dict[direction])[1] < 0:
                    
                    (dict[direction])[3] = "to the left" 
                elif (dict[direction])[1] > 0:
                    (dict[direction])[3] = "to the right"
                (dict[direction])[1] = abs((dict[direction])[1])

        for direction in ["rotY", "velY"]:
            if (dict[direction])[1] != self.defaultValue:
                if (dict[direction])[1] < 0:
                    (dict[direction])[3] = "backwards" 
                elif (dict[direction])[1] > 0:
                    (dict[direction])[3] = "forwards" 
                (dict[direction])[1] = abs((dict[direction])[1])
                
        return dict

    # delete a widget in the grid at (row, col), if it exists
    def DeleteGridWidget(self, row, col):
        item = self.grid.itemAtPosition(row,col)
        if item is not None:
            widget = item.widget()
            if widget is not None:
                self.grid.removeWidget(widget)
                widget.deleteLater()

# Main GUI class that provides the framework for displaying the navdata
class FlightstatsDisplay(QtGui.QMainWindow):

    def __init__(self):
        
        super(FlightstatsDisplay,self).__init__()

        exitAction = QtGui.QAction('&Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.triggered.connect(self.close)


        
        self.imperialAction = QtGui.QAction('&Imperial', self)
        self.metricAction = QtGui.QAction('&Metric', self)
        self.freezeAction = QtGui.QAction('&Freeze', self)
        self.lowAction = QtGui.QAction('&Low', self)
        self.mediumAction = QtGui.QAction('&Medium', self)
        self.highAction = QtGui.QAction('&High', self)
        self.realTimeAction = QtGui.QAction('&Real Time', self)
        self.realTimeAction.triggered.connect(functools.partial(self.ToggleRefreshRate, REAL_TIME_FREQ))

        # initalizing menus
        menubar = self.menuBar()
        menubar.setNativeMenuBar(False)
        fileMenu= menubar.addMenu('&File')
        fileMenu.addAction(exitAction)

        self.viewMenu = menubar.addMenu('&View')
        self.measurementSubmenu = self.viewMenu.addMenu("&System of Measurement")
        self.measurementSubmenu.addAction(self.imperialAction)
        self.measurementSubmenu.addAction(self.metricAction)
        self.refreshRateSubmenu= self.viewMenu.addMenu("&Refresh Rate")
        self.refreshRateSubmenu.addAction(self.freezeAction)
        self.refreshRateSubmenu.addAction(self.lowAction)
        self.refreshRateSubmenu.addAction(self.mediumAction)
        self.refreshRateSubmenu.addAction(self.highAction)
        self.refreshRateSubmenu.addAction(self.realTimeAction)

        self.refreshRateGroup = QtGui.QActionGroup(self)

        # initalizing and showing window
        self.setGeometry(300,300,450,450)
        self.setWindowTitle("AR Drone Flight Info")
        self.mainWidget = MainGridWidget()
        self.setCentralWidget(self.mainWidget)
        
        global curr_view
        curr_view = SIMPLE_VIEW
        self.show()
        
    def ToggleUnits(self, newUnits):
        global curr_units_system
        curr_units_system = newUnits

    def ToggleRefreshRate(self, newRate):
        global curr_refresh_rate
        curr_refresh_rate = newRate

    def ToggleView(self, newView):
        global curr_view
        cur_view = newView


if __name__=="__main__":
    rospy.init_node('FlightstatsDisplay')
    app = QtGui.QApplication(sys.argv)
    display=FlightstatsDisplay()
    sys.exit(app.exec_())
