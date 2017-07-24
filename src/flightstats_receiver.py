#!/usr/bin/env python

import rospy
import collections

from ardrone_autonomy.msg import Navdata


class FlightstatsReceiver(object):
    

    def __init__(self):

        super(FlightstatsReceiver, self).__init__()

        # Subscribe to the navdata topic, and call self.ProcessNavdata whenever 
        # update navdata messages are recieved
        self.navdataSub = rospy.Subscriber('/ardrone/navdata', Navdata, self.UpdateNavdata)

        # dictionary will hold a useful subset of available flight info
        # Key is the variable name; value is (description, value, units, (optional) direction string following units)
        self.defaultValue = "?"
        self.flightInfo = collections.OrderedDict()
        self.flightInfo["batteryPercent"]=["Battery Left: ", self.defaultValue, "%", ""]
        self.flightInfo["state"]=["Status: ", self.defaultValue, "", ""]
        self.flightInfo["altitude"]=["Altitude: ", self.defaultValue, "mm", ""]
        self.flightInfo["rotX"]=["Left/Right Tilt: ", self.defaultValue, u'\N{DEGREE SIGN}', ""]
        self.flightInfo["rotY"]=["Front/Back Tilt: ", self.defaultValue, u'\N{DEGREE SIGN}', ""]
        self.flightInfo["rotZ"]=["Rotation Amount: ", self.defaultValue, u'\N{DEGREE SIGN}', ""]
        self.flightInfo["velX"]=["Left/Right Velocity: ", self.defaultValue, "in/s", ""]
        self.flightInfo["velY"]=["Front/Back Velocity: ", self.defaultValue, "in/s", ""]
        self.flightInfo["velZ"]=["Up/Down Velocity: ", self.defaultValue, "in/s", ""]

    
    # update dictionary as new info from drone comes
    def UpdateNavdata(self, navdata):
        # first, update instance variables
        (self.flightInfo["batteryPercent"])[1]=navdata.batteryPercent
        (self.flightInfo["state"])[1]=navdata.state
        (self.flightInfo["altitude"])[1]=navdata.altd
        (self.flightInfo["rotX"])[1]=navdata.rotX
        (self.flightInfo["rotY"])[1]=navdata.rotY
        (self.flightInfo["rotZ"])[1]=navdata.rotZ
        (self.flightInfo["velX"])[1]=navdata.vx
        (self.flightInfo["velY"])[1]=navdata.vy
        (self.flightInfo["velZ"])[1]=navdata.vz
        

 
