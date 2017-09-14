#!/usr/bin/env python

import rospy
import collections
import cv2
from cv_bridge import CvBridge, CvBridgeError

from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.msg import navdata_altitude
from processing_functions.process_video import ProcessVideo
from sensor_msgs.msg import Image

class FlightstatsReceiver(object):
    

    def __init__(self):

        super(FlightstatsReceiver, self).__init__()

        # Subscribe to the navdata topic, and call self.ProcessNavdata whenever 
        # update navdata messages are recieved
        self.navdataSub = rospy.Subscriber('/ardrone/navdata', Navdata, self.UpdateNavdata)
        self.altitudeSub = rospy.Subscriber('/ardrone/navdata_altitude', navdata_altitude, self.UpdateAltitude)
        self.video=rospy.Subscriber('/ardrone/image_raw', Image, self.VideoUpdate )

        # dictionary will hold a useful subset of available flight info
        # Key is the variable name; value is (description, value, units, (optional) direction string following units)
        self.defaultValue = "?"
        self.flightInfo = collections.OrderedDict()
        self.flightInfo["batteryPercent"]=["Battery Left: ", self.defaultValue, "%", ""]
        self.flightInfo["state"]=["Status: ", self.defaultValue, "", ""]
        self.flightInfo["altitude"]=["Drone Altitude: ", self.defaultValue, "mm", ""]

        self.flightInfo["SVCLAltitude"] = ["SVCL Altitude: ", -1, "mm", ""]
        self.flightInfo["center"] = ["Platform Center: ", self.defaultValue, "", ""]

        self.flightInfo["rotX"]=["Left/Right Tilt: ", self.defaultValue, u'\N{DEGREE SIGN}', ""]
        self.flightInfo["rotY"]=["Front/Back Tilt: ", self.defaultValue, u'\N{DEGREE SIGN}', ""]
        self.flightInfo["rotZ"]=["Rotation Amount: ", self.defaultValue, u'\N{DEGREE SIGN}', ""]

        self.flightInfo["velY"]=["Left/Right Velocity: ", self.defaultValue, "mm/s", ""]
        self.flightInfo["velX"]=["Forwards/Backwards Velocity: ", self.defaultValue, "mm/s", ""]
        self.flightInfo["velZ"]=["Up/Down Velocity: ", self.defaultValue, "mm/s", ""]

        self.flightInfo["accelZ"]=["Up/Down Acceleration: ", self.defaultValue, "mm/s", ""]

        self.flightInfo["dispLR"]=["Left/Right Displacement: ", self.defaultValue, "mm", ""]
        self.flightInfo["dispFB"]=["Forwards/Backwards Displacement: ", self.defaultValue, "mm", ""]
        self.flightInfo["dispUD"]=["Up/Down Displacement: ", self.defaultValue, "mm", ""]

        self.flightInfo["segImage"] = [None]

        self.LRDisplacement = 0.0
        self.FBDisplacement = 0.0
        self.UDDisplacement = 0.0
        self.oldTime = rospy.Time.now()
        self.bridge = CvBridge()
        self.processVideo = ProcessVideo()

        # sometimes the altitude doesn't start at 0; "zero balances" the drone such that where it started is considered 0 altitude
        self.zeroBalanced = False
        self.zeroAltitude = 0

        # counter is designed to throttle the lag that comes with executing videoupdate too often
        # a higher videoUpdateMax will make everything run faster, but getting height/center updates will be slower
        # describes a ratio: compute/rest 
        self.computeMax = 1
        self.restMax = 0
        self.counter = 0


    def VideoUpdate(self, image):
        
        if (self.counter < self.computeMax) or self.restMax == 0:
            
            self.counter +=1

            # converting to hsv
            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            segImage, radius, center = self.processVideo.DetectShape(image, 'orange')

            if radius == None:
                if center == (None,None):
                    distance = -1
                    (self.flightInfo["SVCLAltitude"])[1] = distance
            else:
                distance = self.processVideo.CalcDistanceNew(88, radius* 2)
                (self.flightInfo["SVCLAltitude"])[1] = distance

            (self.flightInfo["center"])[1] = center 
            (self.flightInfo["segImage"]) = segImage

        else:

            if self.counter < self.computeMax + self.restMax:
                self.counter +=1

            if self.counter >= self.computeMax + self.restMax :
                self.counter = 0
            

    def UpdateAltitude(self, altitude):
        if self.zeroBalanced:
            (self.flightInfo["altitude"])[1] = altitude.altitude_raw - self.zeroAltitude
        else:
            self.zeroBalanced = True
            self.zeroAltitude = altitude.altitude_raw

        
    # update dictionary as new info from drone comes
    def UpdateNavdata(self, navdata):

        # first, update instance variables
        (self.flightInfo["batteryPercent"])[1] = navdata.batteryPercent
        (self.flightInfo["state"])[1] = navdata.state
        #(self.flightInfo["altitude"])[1] = navdata.altd
        (self.flightInfo["rotX"])[1] = navdata.rotX
        (self.flightInfo["rotY"])[1] = navdata.rotY
        (self.flightInfo["rotZ"])[1] = navdata.rotZ
        (self.flightInfo["velX"])[1] = navdata.vx
        (self.flightInfo["velY"])[1] = navdata.vy
        (self.flightInfo["velZ"])[1] = navdata.vz
        (self.flightInfo["accelZ"])[1] = navdata.az

        
        dt = rospy.Time.now() - self.oldTime
        # Calculating horizontal displacement
        currLRVelocity = self.flightInfo["velY"][1]
        self.LRDisplacement += float(currLRVelocity) * dt.to_sec()
        (self.flightInfo["dispLR"])[1]=self.LRDisplacement

        # Calculating vertical displacement
        currFBVelocity = self.flightInfo["velX"][1]
        self.FBDisplacement += float(currFBVelocity) * dt.to_sec()
        (self.flightInfo["dispFB"])[1]=self.FBDisplacement

        # Calculating Z displacement
        currUDAcceleration = self.flightInfo["accelZ"][1]
        self.UDDisplacement += float(currUDAcceleration * dt.to_sec() * dt.to_sec())
        (self.flightInfo["dispUD"])[1]=self.UDDisplacement

        self.oldTime = rospy.Time.now()

 
