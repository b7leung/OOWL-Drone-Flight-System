#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import numpy
import cv2
import math

# This directive will give directions that attempt to push the drone
# back towards the target location if it has drifted out of the cameras vision
class ReturnToLineDirective(AbstractDroneDirective):


    # sets up this directive
    # platformColor: color of the platform to return to
    def __init__(self, lineColor, speedModifier = 0.5, radiusThresh = 255):

        self.lineColor = lineColor
        self.processVideo = ProcessVideo()
        self.speedModifier = speedModifier
        self.radiusThresh = radiusThresh
        self.moveTime = 0.25
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
    #   0 if algorithm is still running and drone hasn't returned to the line yet
    #   1 if algorithm is finished and drone is now over the color
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        lines, image = self.processVideo.MultiShowLine(segLineImage)

        #navdata stores the last location and angle in the case of an error
        cx = navdata[1][0][0]
        cy = navdata[1][0][1]     
        angle = navdata[1][1]

        #cv2.circle(image, (cx,cy), self.radiusThresh, (0,255,0), 1)
        
        hasPlatform = False
        # thresh in degrees
        thresh = 18 
        for line in lines:
            if line!=None:
                # original line was found if angle matches original, to some threshold
                if ( 
                (abs(angle - line[0]) < thresh or abs(angle - line[0]) > (180 - thresh)) ):
                    hasPlatform = True
                    cv2.circle(image, line[1], 15, (0,255,0), -1)
                    cx = line[1][0]
                    cy = line[1][1]
                else:
                    cv2.circle(image, line[1], 15, (0,0,255), 5)
 
        if hasPlatform:
            rospy.logwarn("Returned to " + self.lineColor + " line")
            directiveStatus = 1
            zspeed = 0

        else:
            rospy.logwarn("Returning to "+ self.lineColor + " line")
            directiveStatus = 0
            zspeed = 0.1

        if cx == None or cy == None:
            rospy.logwarn("Returning -- no " + self.lineColor + " detected @ this altitude, increasing altitude")
            return 0, (0,0,0,0.5),image, (cx,cy), 0, 0

        
        xspeed, yspeed, _ = self.processVideo.ApproximateSpeed(image, cx, cy,
        ytolerance = 50, xtolerance = 50)
        
        yspeed = min( yspeed * self.speedModifier, 1 )
        xspeed = min( xspeed * self.speedModifier, 1 )
        rospy.logwarn("X Speed: " + str(xspeed) + " Y Speed: " + str(yspeed))

        self.processVideo.DrawCircle(image ,(cx,cy))

        return directiveStatus, (xspeed, yspeed, 0, zspeed), image, ((cx,cy),angle), self.moveTime, self.waitTime, None
        

