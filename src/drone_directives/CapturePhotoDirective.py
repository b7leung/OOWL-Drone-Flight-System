#!usr/bin/env python

import rospy
from AbstractDroneDirective import *
from processing_functions.picture_manager import PictureManager

# Will save the image passed in
class CapturePhotoDirective(AbstractDroneDirective):

    # sets up this directive
    # picture path: location to save this photo
    def __init__(self, picturePath):

        self.pictureManager = PictureManager(picturePath)
    

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the picture has been saved
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        pictureName = self.pictureManager.Capture(image, navdata["altitude"])
        rospy.logwarn("Saved picture as " + pictureName)

        return 1, (0, 0, 0, 0), image, (None, None)

