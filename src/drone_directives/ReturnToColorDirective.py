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
    def __init__(self, platformColor):

        self.platformColor = platformColor
        self.processVideo = ProcessVideo()
        self.lastLocation = (None, None)


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
        
        
        """platform_image = self.processVideo.DetectColor(image, self.platformColor)
        numrows,numcols,channels=self.image.shape
        
        cx = self.lastLocation[0]
        cy = self.lastLocation[1]

        centerx=numcols/2
        centery=numrows/2
        width=120
        height=120
        xlower=centerx-width #left xvalue
        ylower=centery-height #"top" yvalue
        xupper=centerx+width #right xvalue
        yupper=centery+height #"bottom" yvalue
        hasPlatform = self.process.IsHueDominant(platform_image, 0, 360, 0.2)   

        if xspeed == 0 and yspeed == 0 and zspeed == 0 and yawspeed == 0 and cx != None and cy != None:

            rospy.logwarn("Vertically facing " + self.lineColor + " line")
            directiveStatus = 1

        else:

            rospy.logwarn("Trying to vertically face " + self.lineColor + " line")
            directiveStatus = 0 

        return directiveStatus, (xspeed, yspeed, yawspeed, zspeed), platform_image
        """
        pass




