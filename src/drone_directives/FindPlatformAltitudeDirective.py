#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *


# Will make the drone up to a maximum altitude in hopes of finding the platform
class FindPlatformAltitudeDirective(AbstractDroneDirective):


    # sets up this directive
    # maxaltitude is in mm
    def __init__(self, platformColor, maxAltitude):

        self.platformColor = platformColor
        self.maxAltitude = maxAltitude
        self.processVideo = ProcessVideo()


    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone has found the platform
    #   0 if the algorithm is still running and is increasing height since platform hasn't been found yet
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        currAltitude = navdata["altitude"][1]
        climbspeed = 1

        if currAltitude < self.maxAltitude: 
            zVel = climbspeed
            rospy.logwarn("Drone is increasing altitude")
        
        else:
            zVel = 0
        
        # Check if the platform is visible
        platform_image = self.processVideo.DetectColor(image, self.platformColor)
        hasPlatform = self.processVideo.IsHueDominant(platform_image, 0, 360, 0.1)  

        if hasPlatform:
            rospy.logwarn("Drone has found platform; stopped height increase")
            directiveStatus = 1
            zVel = 0
        else:
            directiveStatus = 0
            if zVel == 0:
                rospy.logwarn("Reached maxiumum altitude, still no platform")

        return directiveStatus, (0, 0, 0, zVel), platform_image, (None,None)

