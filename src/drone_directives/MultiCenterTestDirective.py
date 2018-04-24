#!usr/bin/env python

import rospy
import cv2
from AbstractDroneDirective import *
from processing_functions.process_video import ProcessVideo

class MultiCenterTestDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self, color):
        
        self.color = color
        self.processVideo = ProcessVideo()

        
    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 as long as the drone is idle
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        image = self.processVideo.RemoveNoise(navdata["segImage"])
        alt = int(navdata["SVCLAltitude"][1])
        rospy.logwarn("Curr Altitude = " + str(alt))
        if alt == -1:
            rospy.logwarn("**************************************************")
            rospy.logwarn("asdfasdf")

        
        cx, cy = navdata["center"][1][0], navdata["center"][1][1]

        if cx != None and cy != None:
            cv2.circle(image, (cx,cy), 6, (255,255,255), -1)

            
        return 1, (0, 0, 0, 0), image, (None, None), 0, 0, None

