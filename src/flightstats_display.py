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
curr_view = None

# frequency to update GUI display, in ms
FREEZE_FREQ = -1
LOW_FREQ= 250
MEDIUM_FREQ=130
HIGH_FREQ = 10
REAL_TIME_FREQ = 2
curr_refresh_rate = None

# units to display in
IMPERIAL_UNITS = 0
METRIC_UNITS = 1
curr_units_system = None

# A Widget that provides all the navadata recieved as a flightstatsreceiver
# in a easy to view grid layout 
class MainGridWidget(FlightstatsReceiver, QtGui.QWidget):

    def __init__(self):
        super(MainGridWidget,self).__init__()

        # setting up grid
        self.grid = QtGui.QGridLayout()
        self.UpdateGridInfo(True)
        self.setLayout(self.grid)

        # Start timer that will refresh the widget every specified number of ms
        self.updateTimer = QtCore.QTimer(self)
        self.updateTimer.timeout.connect(self.UpdateGridInfo)
        self.updateTimer.start(curr_refresh_rate)
    
    
    def UpdateTimer(self):
        if curr_refresh_rate == FREEZE_FREQ:
            self.updateTimer.stop()
        else:
            # if timer was frozen, start it up again
            self.updateTimer.start(curr_refresh_rate)


    # Updates GUI with the latest information in the Dictionary
    def UpdateGridInfo(self, initial = False):

        # process dictionary values first so it's easy to read
        processedDict = self.ProcessValues()
        
        # update grid with processed values
        row = 0
        for key, value in processedDict.iteritems():
            # entries are that aren't meant to be displayed are typed below
            if key != "segImage":
                
                newText = str(value[1])+ " " + value[2] + " " +  value[3]
                # addingcurrent data
                if initial is True:
                    self.grid.addWidget(QtGui.QLabel(value[0]), row,0, QtCore.Qt.AlignCenter)
                    self.grid.addWidget(QtGui.QLabel(newText),row, 1, QtCore.Qt.AlignCenter)
                else:
                    self.grid.itemAtPosition(row,1).widget().setText(newText)
                row +=1
                if initial is True:
                    line = QtGui.QFrame()
                    line.setFrameShape(QtGui.QFrame.Shape.HLine)
                    self.grid.addWidget(line, row, 0, 1, 2)
                row +=1
        

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

        # Converting altitude from mm
        for alt in ["SVCLAltitude", "altitude" ]:
            if (dict[alt])[1] != self.defaultValue:
                if curr_units_system == METRIC_UNITS:
                    # already in mm
                    pass
                elif curr_units_system == IMPERIAL_UNITS:
                    #converting from mm to m
                    (dict[alt])[1] = (dict[alt])[1] / 1000.0
                    #converting from M to in
                    (dict[alt])[1] = 12*((dict[alt])[1] / 0.3048)
                    (dict[alt])[2] = "in"
        
        # Convert velocity from mm/s to in/s
        for vel in ["velX", "velY", "velZ"]:
            if (dict[vel])[1] != self.defaultValue:
                if curr_units_system == METRIC_UNITS:
                    pass
                elif curr_units_system == IMPERIAL_UNITS:
                    # converting from mm/s to in/s
                    (dict[vel])[1] = (dict[vel])[1] * 0.0393701
                    (dict[vel])[2] = "in/s"

        # rounding all values to make it easier to view
        for num in ["altitude", "rotX", "rotY", "rotZ", "velX", "velY",
        "velZ", "dispLR", "dispFB", "dispUD", "accelZ", "SVCLAltitude"]:
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
        for direction in ["rotX"]:
            if (dict[direction])[1] != self.defaultValue:
                if (dict[direction])[1] < 0:
                    
                    (dict[direction])[3] = "to the left" 
                elif (dict[direction])[1] > 0:
                    (dict[direction])[3] = "to the right"
                (dict[direction])[1] = abs((dict[direction])[1])

        # Add corresponding direction strings
        for direction in ["velY","dispLR"]:
            if (dict[direction])[1] != self.defaultValue:
                if (dict[direction])[1] > 0:
                    
                    (dict[direction])[3] = "to the left" 
                elif (dict[direction])[1] < 0:
                    (dict[direction])[3] = "to the right"
                (dict[direction])[1] = abs((dict[direction])[1])


        for direction in ["rotY", "velX", "dispFB"]:
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
        
        # setting up menu actions
        exitAction = QtGui.QAction('&Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.triggered.connect(self.close)


        # units actions 
        self.imperialAction = QtGui.QAction('&Imperial', self, checkable = True)
        self.imperialAction.triggered.connect(functools.partial(self.ToggleUnits, IMPERIAL_UNITS))
        self.metricAction = QtGui.QAction('&Metric', self, checkable = True)
        self.metricAction.triggered.connect(functools.partial(self.ToggleUnits, METRIC_UNITS))

        self.unitsGroup = QtGui.QActionGroup(self)
        self.unitsGroup.addAction(self.imperialAction)
        self.unitsGroup.addAction(self.metricAction)
        self.metricAction.setChecked(True)

        # refresh rate actions
        self.freezeRateAction = QtGui.QAction('&Freeze', self, checkable = True)
        self.freezeRateAction.triggered.connect(functools.partial(self.ToggleRefreshRate, FREEZE_FREQ))
        self.lowRateAction = QtGui.QAction('&Low', self, checkable = True )
        self.lowRateAction.triggered.connect(functools.partial(self.ToggleRefreshRate, LOW_FREQ))
        self.mediumRateAction = QtGui.QAction('&Medium', self, checkable = True)
        self.mediumRateAction.triggered.connect(functools.partial(self.ToggleRefreshRate, MEDIUM_FREQ))
        self.highRateAction = QtGui.QAction('&High', self, checkable = True)
        self.highRateAction.triggered.connect(functools.partial(self.ToggleRefreshRate, HIGH_FREQ))
        self.realTimeRateAction = QtGui.QAction('&Real Time', self, checkable = True)
        self.realTimeRateAction.triggered.connect(functools.partial(self.ToggleRefreshRate, REAL_TIME_FREQ))

        self.refreshRateGroup = QtGui.QActionGroup(self)
        self.refreshRateGroup.addAction(self.freezeRateAction)
        self.refreshRateGroup.addAction(self.lowRateAction)
        self.refreshRateGroup.addAction(self.mediumRateAction)
        self.refreshRateGroup.addAction(self.highRateAction)
        self.refreshRateGroup.addAction(self.realTimeRateAction)
        self.highRateAction.setChecked(True)

        # accuracy view actions
        self.fullViewAction = QtGui.QAction("&Full View", self, checkable = True)
        self.fullViewAction.triggered.connect(functools.partial(self.ToggleView, FULL_VIEW))
        self.mediumViewAction = QtGui.QAction("&Medium View", self, checkable = True)
        self.mediumViewAction.triggered.connect(functools.partial(self.ToggleView, MEDIUM_VIEW))
        self.simpleViewAction = QtGui.QAction("&Simple View", self, checkable = True)
        self.simpleViewAction.triggered.connect(functools.partial(self.ToggleView, SIMPLE_VIEW))

        self.accuracyViewGroup = QtGui.QActionGroup(self)
        self.accuracyViewGroup.addAction(self.fullViewAction)
        self.accuracyViewGroup.addAction(self.mediumViewAction)
        self.accuracyViewGroup.addAction(self.simpleViewAction)
        self.simpleViewAction.setChecked(True)

        # initalizing menus
        menubar = self.menuBar()
        menubar.setNativeMenuBar(False)

        fileMenu= menubar.addMenu('&File')
        fileMenu.addAction(exitAction)

        self.viewMenu = menubar.addMenu('&View')
        self.measurementSubmenu = self.viewMenu.addMenu("&System of Measurement")
        self.measurementSubmenu.addAction(self.imperialAction)
        self.measurementSubmenu.addAction(self.metricAction)
        self.refreshRateSubmenu = self.viewMenu.addMenu("&Refresh Rate")
        self.refreshRateSubmenu.addAction(self.freezeRateAction)
        self.refreshRateSubmenu.addAction(self.lowRateAction)
        self.refreshRateSubmenu.addAction(self.mediumRateAction)
        self.refreshRateSubmenu.addAction(self.highRateAction)
        self.refreshRateSubmenu.addAction(self.realTimeRateAction)
        self.accuracySubmenu = self.viewMenu.addMenu("&Decimal Accuracy")
        self.accuracySubmenu.addAction(self.fullViewAction)
        self.accuracySubmenu.addAction(self.mediumViewAction)
        self.accuracySubmenu.addAction(self.simpleViewAction)

        # setting defaults
        global curr_view
        curr_view = SIMPLE_VIEW
        global curr_refresh_rate
        curr_refresh_rate = HIGH_FREQ
        global curr_units_system
        curr_units_system = METRIC_UNITS

        # initalizing and showing window
        self.setGeometry(300,300,450,450)
        self.setWindowTitle("AR Drone Flight Info")
        self.mainWidget = MainGridWidget()
        self.setCentralWidget(self.mainWidget)
        
        self.show()
        
    def ToggleUnits(self, newUnits):
        global curr_units_system
        curr_units_system = newUnits

    def ToggleRefreshRate(self, newRate):
        global curr_refresh_rate
        curr_refresh_rate = newRate
        self.mainWidget.UpdateTimer()

    def ToggleView(self, newView):
        global curr_view
        curr_view = newView


if __name__=="__main__":
    rospy.init_node('FlightstatsDisplay')
    app = QtGui.QApplication(sys.argv)
    display=FlightstatsDisplay()
    sys.exit(app.exec_())
