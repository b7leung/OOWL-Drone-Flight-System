#!/usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import cv2

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

        height, _, _ = image.shape
        
        segImage = self.processVideo.DetectColor(image, self.lineColor)
        angle = self.processVideo.ShowLine(segImage, 60, 130,thresh = 40)

        cx, cy = self.processVideo.CenterOfMass(segImage)

        _, yspeed, _ = self.processVideo.ApproximateSpeed(segImage, cx, cy, 
        navdata["altitude"][1], 0, xtolerance = 50, ytolerance = 50)
        #navdata["altitude"][1], 0, tolerance = int((height*0.8)/2))

        if angle == None:

            xspeed = 0
            yspeed = 0
            directiveStatus = 1
            rospy.logwarn("Finished following line")
            
        else:

            xspeed = -0.9
            directiveStatus = 0

        if yspeed !=0:
            rospy.logwarn("Moving blue line back to center")
            xspeed = 0
        elif xspeed != 0:
            rospy.logwarn("Drone just going forward")


        return directiveStatus, (xspeed, yspeed, 0, 0), segImage, (None, None)



