#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class HoverColorDirective(AbstractDroneDirective):
    
    # sets up this directive
    # platformColor: color to hover over
    # hoverAltitude: how high to hover over the color, in mm
    def __init__(self, platformColor, hoverAltitude):

        self.platformColor = platformColor 
        self.hoverAltitude = hoverAltitude
        self.processVideo = ProcessVideo()
        self.moveTime = 0
        self.waitTime = 0.10
    

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone isn't on orange yet
    #   1 if algorithm is finished and drone is now on orange
    #  -1 if an error has occured
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    #
    # A coordinate pair that represents where things are being tracked
    # If nothing is being tracked, (None,None) is returned

    def RetrieveNextInstruction(self, image, navdata):
        
        # color segmented image
        segImage = self.processVideo.DetectColor(image, self.platformColor)
        cx, cy = self.processVideo.CenterOfMass(segImage)
        xspeed, yspeed, zspeed = self.processVideo.ApproximateSpeed(segImage, cx, cy,
        navdata["altitude"][1], self.hoverAltitude, xtolerance = 55, ytolerance = 55)

        # if there is orange in the screen, and the drone is in the middle, return true
        if cx != None and cy != None and xspeed == 0 and yspeed == 0 and zspeed == 0:

            rospy.logwarn("Done Hovering on " + self.platformColor)
            directiveStatus = 1

        elif cx == None or cy == None:
            
            rospy.logwarn("Can't Hover on " + self.platformColor + "; lost platform")
            directiveStatus = -1

        else:

            rospy.logwarn("Trying to Hover on " + self.platformColor)
            directiveStatus = 0

        return directiveStatus, (xspeed, yspeed, 0, zspeed), segImage, (cx,cy), self.moveTime, self.waitTime, None



