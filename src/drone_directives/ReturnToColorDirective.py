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

        #navdata stores the last location in the case of an error
        cx = navdata[0]
        cy = navdata[1]     
        if cx == None or cy == None:
            return -1, (0,0,0,0),image,navdata
        platform_image = self.processVideo.DetectColor(image, self.platformColor)
        #Draws a circle over the last location of the drone
        self.processVideo.DrawCircle(platform_image,(cx,cy))

        numrows,numcols,channels=platform_image.shape

        centerx=numcols/2
        centery=numrows/2
        width=90
        height=90
        xlower=centerx-width #left xvalue
        ylower=centery-height #"top" yvalue
        xupper=centerx+width #right xvalue
        yupper=centery+height #"bottom" yvalue
        hasPlatform = self.processVideo.IsHueDominant(platform_image, 0, 360, 0.2)   
        
        #if the last location for the object was in the center of image do nothing
        #if ((cx > xlower and cx < xupper) or (cy > ylower and cy < yupper)):
        #    return -1, (0, 0, 0, 0), platform_image, navdata
     
        xspeed, yspeed, _ = self.processVideo.ApproximateSpeed(platform_image, cx, cy)
        return 0,(xspeed,yspeed, 0, 0), platform_image, navdata
        




