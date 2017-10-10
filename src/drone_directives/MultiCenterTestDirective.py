#!usr/bin/env python

import rospy
import cv2
from AbstractDroneDirective import *
from processing_functions.process_video import ProcessVideo

class MultiCenterTestDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self):

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

        image = self.processVideo.DetectColor(image, "orange")
        centers, image = self.processVideo.MultiCenterOfMass(image)
        
        for c in centers:
            cv2.circle(image, c, 10, (0,255,255), -1)
            
        return 1, (0, 0, 0, 0), image, (None, None), 0, 0
