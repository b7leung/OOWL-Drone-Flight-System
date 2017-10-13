#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *


# Will make the drone return to where it took off
# given some platform color
class ReturnToOriginDirective(AbstractDroneDirective):


    # sets up this directive
    # tolerance is in mm; how close it needs to be from where it took off
    def __init__(self, platformColor, tolerance):

        self.platformColor = platformColor
        self.tolerance = tolerance
        self.processVideo = ProcessVideo()


    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone has reached where it took off
    #   0 if the algorithm is still running and the drone hasn't reached where it took off
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        dispH = navdata["dispLR"][1]
        
        speed = 0.2
    
        # calculating horizontal
        if dispH < self.tolerance * -1:
            
            roll = speed
            rospy.logwarn("Drone is returning to origin; going left")

        elif dispH > self.tolerance:

            roll = speed * -1
            rospy.logwarn("Drone is returning to origin; going right")
        
        else:
            roll = 0

        # calculating vertical 
        dispV = navdata["dispFB"][1]
        if dispV < self.tolerance * -1:
            rospy.logwarn("Drone is returning to origin; going forward")
            pitch = speed

        elif dispV > self.tolerance:
            rospy.logwarn("Drone is returning to origin; going backwards")
            pitch = speed * -1

        else:
            pitch = 0
        
        # check if the platform is visible
        platform_image = self.processVideo.DetectColor(image, self.platformColor)
        hasPlatform = self.processVideo.IsHueDominant(platform_image, 0, 360, 0.2)   

        if (pitch == 0 and roll == 0) or hasPlatform:
            directiveStatus = 1
            rospy.logwarn("Drone has returned to origin")
        else:
            directiveStatus = 0


        return directiveStatus, (roll, pitch, 0, 0), image, (None,None), 0, 0, None

