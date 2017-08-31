#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import numpy

# This directive will give directions that attempt to push the drone
# back towards the target location if it has drifted out of the cameras vision
class ReturnToColorDirective(AbstractDroneDirective):


    # sets up this directive
    # platformColor: color of the platform to return to
    def __init__(self, platformColor, speedModifier = 0.5):

        self.platformColor = platformColor
        self.processVideo = ProcessVideo()
        self.speedModifier = speedModifier
    

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
        
        #navdata stores the last location in the case of an error
        cx = navdata[1][0]
        cy = navdata[1][1]     


        if navdata[0]["center"][1][0] != None and navdata[0]["center"][1][1] != None:
            hasPlatform = True
        else:
            hasPlatform = False

        #platform_image = self.processVideo.DetectColor(image, self.platformColor)
        #cropped = self.processVideo.CropVisible(platform_image, 50,50, 500, 220)
        #hasPlatform = self.processVideo.IsHueDominant(platform_image, 0, 360, 0.1)   

        if hasPlatform:
            cx, cy = navdata[0]["center"][1][0], navdata[0]["center"][1][1]
            rospy.logwarn("Returned to platform")
            directiveStatus = 1
            zspeed = 0

        else:
            rospy.logwarn("Returning to platform")
            directiveStatus = 0
            zspeed = 0.5

        if cx == None or cy == None:
            rospy.logwarn("Returning -- no orange detected @ this altitude, increasing altitude")
            return 0, (0,0,0,0.5),image, (cx,cy)

        
        xspeed, yspeed, _ = self.processVideo.ApproximateSpeed(image, cx, cy,
        ytolerance = 50, xtolerance = 50)
        
        rospy.logwarn("X Speed: " + str(xspeed) + " Y Speed: " + str(yspeed))

        self.processVideo.DrawCircle(image,(cx,cy))

        return directiveStatus, (xspeed*self.speedModifier, yspeed*self.speedModifier, 0, zspeed), image, (cx,cy)
        




