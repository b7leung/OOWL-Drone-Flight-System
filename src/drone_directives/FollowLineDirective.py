#!/usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order to 
# follow a line below it, given that the drone is perpendicular to the line
class FollowLineDirective(AbstractDroneDirective):
    
    # sets up this directive
    # lineColor: color of the line to follow
    def __init__(self, lineColor):

        self.lineColor = lineColor
        self.processVideo = ProcessVideo()


    # Given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone is still following line
    #   1 if algorithm is finished and has finished following line
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        segImage = self.processVideo.DetectColor(image, self.lineColor)

        angle = self.processVideo.ShowLine(segImage, 60, 120)
        
        if angle == None:

            rospy.logwarn("Done following " + self.lineColor + " line")
            xspeed = 0
            directiveStatus = 1
            
        else:

            rospy.logwarn("Trying to follow " + self.lineColor + " line")
            xspeed = -0.3
            directiveStatus = 0

        return directiveStatus, (xspeed, 0, 0, 0), segImage, (None, None)



