#!usr/bin/env python

import rospy
import cv2
from AbstractDroneDirective import *
from processing_functions.process_video import ProcessVideo


# Will toggle the drone's camera
class BinaryTestDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self):
        
        self.processVideo = ProcessVideo()
    

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone's camera has been toggled
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        segmented, hsv, binary = self.processVideo.DetectColor(image, 'orange', "all")
        image = binary

        cx, cy = navdata["center"][1][0], navdata["center"][1][1]
        if cx != None and cy != None:
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 

        return 1, (0, 0, 0, 0), image, (None,None)

