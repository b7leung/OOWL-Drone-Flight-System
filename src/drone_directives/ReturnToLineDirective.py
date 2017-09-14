#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import numpy

# This directive will give directions that attempt to push the drone
# back towards the target location if it has drifted out of the cameras vision
class ReturnToLineDirective(AbstractDroneDirective):


    # sets up this directive
    # platformColor: color of the platform to return to
    def __init__(self, lineColor, speedModifier = 0.5):

        self.lineColor = lineColor
        self.processVideo = ProcessVideo()
        self.speedModifier = speedModifier
    

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
        
        segLineImage, _, _ = self.processVideo.DetectColor(image, self.lineColor, "all")

        line1Angle, line1Center, line2Angle, line2Center = self.processVideo.ShowTwoLines(segLineImage)

        #navdata stores the last location in the case of an error
        cx = navdata[1][0]
        cy = navdata[1][1]     


        if line1Angle != None:
            hasPlatform = True
        else:
            hasPlatform = False
        if hasPlatform:
            cx, cy = line1Center[0], line1Center[1]
            rospy.logwarn("Returned to platform")
            directiveStatus = 1
            zspeed = 0

        else:
            rospy.logwarn("Returning to platform")
            directiveStatus = 0
            zspeed = 0.5

        if cx == None or cy == None:
            rospy.logwarn("Returning -- no " + self.lineColor + " detected @ this altitude, increasing altitude")
            return 0, (0,0,0,0.5),image, (cx,cy), 0, 0

        
        xspeed, yspeed, _ = self.processVideo.ApproximateSpeed(image, cx, cy,
        ytolerance = 50, xtolerance = 50)
        
        rospy.logwarn("X Speed: " + str(xspeed) + " Y Speed: " + str(yspeed))

        self.processVideo.DrawCircle(segLineImage,(cx,cy))

        return directiveStatus, (xspeed*self.speedModifier, yspeed*self.speedModifier, 0, zspeed), segLineImage, (cx,cy), 0, 0
        

