#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import numpy
import math
import cv2

# This directive will give directions that attempt to push the drone
# back towards the target location if it has drifted out of the cameras vision
class ReturnToColorDirective(AbstractDroneDirective):


    # sets up this directive
    # platformColor: color of the platform to return to
    def __init__(self, platformColor, speedModifier = 0.5, radiusThresh = 170):

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
    #   0 if algorithm is still running and drone hasn't returned to the color yet
    #   1 if algorithm is finished and drone is now over the color
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        image = self.processVideo.DetectColor(image, self.platformColor)
        centers, _ = self.processVideo.MultiCenterOfMass(image)

        #navdata stores the last location in the case of an error
        cx = navdata[1][0]
        cy = navdata[1][1]  

        if cx == None or cy == None :
            rospy.logwarn("Returning -- no " + self.platformColor + " detected @ this altitude, increasing altitude")
            return 0, (0,0,0,0.5),image, (cx,cy), 0, 0

        cv2.circle(image, (cx,cy), self.radiusThresh, (0,255,0), 1)

        hasPlatform = False

        for c in centers:

            cv2.circle(image, c, 10, (0,255,255), -1)
            if self.InsideCircle( c , (cx,cy), self.radiusThresh):
                hasPlatform = True
                cx, cy = c[0], c[1]
        
        if hasPlatform:
            rospy.logwarn("Returned to platform")
            directiveStatus = 1
            zspeed = 0

        else:
            rospy.logwarn("Returning to platform")
            directiveStatus = 0
            zspeed = 0.5

        xspeed, yspeed, _ = self.processVideo.ApproximateSpeed(image.copy(), cx, cy,
        ytolerance = 50, xtolerance = 50)
        
        rospy.logwarn("X Speed: " + str(xspeed) + " Y Speed: " + str(yspeed))
        
        return directiveStatus, (xspeed*self.speedModifier, yspeed*self.speedModifier, 0, zspeed), image, (cx,cy), self.moveTime, self.waitTime, None
        

