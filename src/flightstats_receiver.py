#!/usr/bin/env python

import rospy

from ardrone_autonomy.msg import Navdata


class FlightstatsReceiver(object):
    

    def __init__(self):

        super(FlightstatsReceiver, self).__init__()
        # Subscribe to the navdata topic, and call self.ProcessNavdata whenever 
        # update navdata messages are recieved
        self.navdataSub = rospy.Subscriber('/ardrone/navdata', Navdata, self.UpdateNavdata)

        # initalize instance variables that will hold 
        # a useful subset of available flight info
        self.batteryPercent = None
        self.state = None
        self.rotX = None
        self.rotY = None
        self.rotZ = None
        self.altitude = None
        self.velX = None
        self.velY = None
        self.velX = None


    def UpdateNavdata(self, navdata):
        # first, update instance variables
        self.batteryPercent = navdata.batteryPercent
        self.ProcessNavdata()
        

    def ProcessNavdata(self):
        pass

 
    def getBatteryPercent(self):
        return self.batteryPercent

