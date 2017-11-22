#!usr/bin/env python

import rospy
import cv2
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import math

# after the drone switches cameras, refers to the last seen platform 
# and makes sure the drone is still over it
class ResumePlatformDirective(AbstractDroneDirective):
    

    # sets up this directive
    # platformColor: color of the platform to return to
    def __init__(self, platformColor, speedModifier = 0.3, radiusThresh = 155):

        self.platformColor = platformColor
        self.processVideo = ProcessVideo()
        self.speedModifier = speedModifier
        self.radiusThresh = radiusThresh
        self.moveTime = 0.20
        self.waitTime = 0.10   


    def InsideCircle(self, point, circleCenter, circleRadius):
        x = point[0]
        y = point[1]
        center_x = circleCenter[0]
        center_y = circleCenter[1]
        radius = circleRadius

        return (math.pow((x-center_x),2) + math.pow((y-center_y),2)) < math.pow(radius,2)


    # Given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone hasn't identified the previous platform
    #   1 if algorithm is finished and drone is now over the previous platform
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        image = navdata["segImage"]
        centers = navdata["allCenters"][1]

        # contains last platform location
        cx = navdata["lastRecordedCenter"][1][0]
        cy = navdata["lastRecordedCenter"][1][1]

        if cx == None or cy == None :
            rospy.logwarn("ERROR")
            return -1, (0,0,0,0.5),image, (cx,cy), 0, 0

        cv2.circle(image, (cx,cy), self.radiusThresh, (0,255,0), 1)
        cv2.circle(image, (cx,cy), 7, (0,255,0), -1)

        resumed = False

        for c in centers:

            cv2.circle(image, c, 10, (0,255,255), -1)
            if self.InsideCircle( c , (cx,cy), self.radiusThresh):
                resumed = True
                cx, cy = c[0], c[1]
        
        if resumed:
            rospy.logwarn("Over last platform")
            directiveStatus = 1
            zspeed = 0

        else:
            rospy.logwarn("Returning to last platform -- increasing altitude")
            directiveStatus = 0
            zspeed = 0.5
        
        #todo: incorrect calculation. need to change center to last, not center
        xspeed, yspeed, _ = self.processVideo.ApproximateSpeed(image.copy(), cx, cy, ytolerance = 0, xtolerance = 0)

        xspeed = xspeed * self.speedModifier
        yspeed = yspeed * self.speedModifier
        
        rospy.logwarn("X Speed: " + str(xspeed) + " Y Speed: " + str(yspeed))
        
        return directiveStatus, (xspeed, yspeed, 0, zspeed), image, (cx,cy), self.moveTime, self.waitTime, None
        

