#!/usr/bin/env python

import rospy
import sys

#from flightstats_receiver import FlightstatsReceiver

from PySide import QtGui


# an extension of FlightstatsReciever that also provides a GUI interface
#class FlightstatsDisplay(FlightstatsReceiver, QtGui.QWidget):
class FlightstatsDisplay(QtGui.QWidget):

    def __init__(self):
        
        super(FlightstatsDisplay,self).__init__()
        self.setGeometry(300,300,250,150)
        self.setWindowTitle("Flight Statistics Display")
        self.show()


        

    def ProcessNavdata(self):
        pass
        #rospy.logwarn(self.getBatteryPercent())


if __name__=="__main__":
    rospy.init_node('FlightstatsDisplay')
    app = QtGui.QApplication(sys.argv)
    display=FlightstatsDisplay()
    sys.exit(app.exec_())
