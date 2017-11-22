#!/usr/bin/env python

import rospy
import collections
import cv2
from cv_bridge import CvBridge, CvBridgeError

from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.msg import navdata_altitude
from processing_functions.process_video import ProcessVideo
from sensor_msgs.msg import Image
import math

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
        self.flightInfo["altitude_raw"]=["Drone Raw Altitude: ", self.defaultValue, "mm", ""]

        self.flightInfo["SVCLAltitude"] = ["SVCL Altitude: ", -1, "mm", ""]
        self.flightInfo["center"] = ["Inferred Center: ", self.defaultValue, "", ""]
        self.flightInfo["lastCenter"] = ["Previous Center: ", (None,None), "", ""]
        self.flightInfo["lastRecordedCenter"] = ["Last Recorded Algorithm Center: ", (None,None), "", ""]
        self.flightInfo["allCenters"] = ["All Centers: ", self.defaultValue, "", ""]

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
        self.lastLocation = (None,None)
        self.lastLoc = (None,None)
        self.lastCenter = (None,None)
        self.lastCenterCount = 0
        self.lastCenterMax = 8

    def VideoUpdate(self, image):
        
        if (self.counter < self.computeMax) or self.restMax == 0:
            
            self.counter +=1

            # converting to hsv
            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            segImage, radius, center = self.processVideo.RecognizeShape(image, 'orange',self.lastLocation)

            (self.flightInfo["center"][1]) = self.InferCenter(self.processVideo.RemoveNoise(segImage))

            if radius == None:
                if center == (None,None):
                    distance = -1
                    (self.flightInfo["SVCLAltitude"])[1] = distance
            else:
                distance = self.processVideo.CalcDistanceNew(88, radius* 2)
                (self.flightInfo["SVCLAltitude"])[1] = distance

            #(self.flightInfo["center"])[1] = center 
            self.lastLocation = center
            (self.flightInfo["segImage"]) = segImage

        else:

            if self.counter < self.computeMax + self.restMax:
                self.counter +=1

            if self.counter >= self.computeMax + self.restMax :
                self.counter = 0
    

    # Uses a more complex way to infer what platform to use. If there is only one platform, this is trivial,
    # but becomes more useful if there are >1 visible.
    def InferCenter(self, image):
        
        centers, _ = self.processVideo.MultiCenterOfMass(image)
        (self.flightInfo["allCenters"][1]) = centers

        if len(centers) == 0:

            #rospy.logwarn("No Platform")
            center = (None,None)

        elif len(centers) == 1:

            center = centers[0]
            #rospy.logwarn("Found 1")

        elif len(centers) == 2:

            if self.lastLoc != (None,None):
                #rospy.logwarn("Found 2 -- picking closer to last")

                # just pick whichever is closer to the last center
                c1Dist = math.sqrt( math.pow((self.lastLoc[1] - centers[0][1]),2) 
                + math.pow((self.lastLoc[0] - centers[0][0]),2 ) ) 

                c2Dist = math.sqrt( math.pow((self.lastLoc[1] - centers[1][1]),2) 
                + math.pow((self.lastLoc[0] - centers[1][0]),2 ) ) 

                if c1Dist < c2Dist:
                    center = centers[0]
                else:
                    center = centers[1]

            else:

                #rospy.logwarn("Found 2 -- picking closer to center")
                # just pick whichever's x-coord is nearer to center
                xCenter = 320
                c1XDist = abs(xCenter - centers[0][0])
                c2XDist = abs(xCenter - centers[1][0])

                if c1XDist < c2XDist:
                    center = centers[0]
                else:
                    center = centers[1]
        
        # if there are 3 or more platforms
        else:

            centers = sorted(centers, key=self.getX)

            if len(centers) % 2 == 0:
                midX = int( (centers[len(centers)/2][0] + centers[(len(centers)/2)+1][0])/2 )
                midY = int( (centers[len(centers)/2][1] + centers[(len(centers)/2)+1][1])/2 )
            else:
                midX = centers[int( (len(centers)/2.0) + 0.5 )-1][0] 
                midY = centers[int( (len(centers)/2.0) + 0.5 )-1][1] 

            center = (midX, midY)

            #rospy.logwarn("Found " + str(len(centers)) + ": " + str(centers))
            #rospy.logwarn("Using " + str(center))

        if len(centers) > 1 and self.lastCenter != (None,None):
            
            closest = None
            smallestLen = None
            # loop to pick whichever is closer to the last center
            for i in range(len(centers)):

                cXDist = abs(self.lastCenter[0] - centers[i][0])
                cYDist = abs(self.lastCenter[1] - centers[i][1])
                length = math.sqrt(math.pow(cXDist,2) + math.pow(cYDist,2) )
                if smallestLen == None or length < smallestLen:
                    closest = centers[i]
                    smallestLen = length

            self.lastCenter = closest
            (self.flightInfo["lastCenter"][1]) = self.lastCenter 
            

        # algorithm waits for a brief moment if last location changes drastically to make
        # sure that it really changed
        if self.lastLoc != (None,None) and center != (None,None):
            dist = math.sqrt( math.pow((self.lastLoc[1] - center[1]),2) 
            + math.pow((self.lastLoc[0] - center[0]),2 ) ) 
            if dist > 140:
                self.lastCenterCount +=1
                rospy.logwarn("visible: " + str(len(centers)) + " change Center Count: " 
                + str(self.lastCenterCount) + " / " + str(self.lastCenterMax))
                if self.lastCenterCount >= self.lastCenterMax:
                    rospy.logwarn("Changing center")
                    self.lastCenterCount = 0
                    # saving last center in case
                    self.lastCenter = self.lastLoc
                    self.lastLoc = center
            else:
                self.lastCenterCount = 0
                self.lastLoc = center
        else:
            self.lastLoc = center


        #return center
        return self.lastLoc


    def getX(self,coordinates):
        return coordinates[0]
    
    # force set what the center is
    def SetCenter(self, center):
        self.lastLoc = center


    def UpdateRecordedCenter(self, center):
        (self.flightInfo["lastRecordedCenter"][1]) = center 


    def UpdateAltitude(self, altitude):
        if self.zeroBalanced:
            (self.flightInfo["altitude_raw"])[1] = altitude.altitude_raw - self.zeroAltitude
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
        (self.flightInfo["altitude"])[1] = navdata.altd
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

 
