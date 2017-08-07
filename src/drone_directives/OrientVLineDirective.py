#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order to orient itself
# vertically with a line underneath it
class OrientVLineDirective(AbstractDroneDirective):


    # sets up this directive
    # lineColor: color of the line to orient vertically with
    # platformColor: color of the platform to orient with
    # hoverAltitude: how high to hover over the platform
    def __init__(self, lineColor, platformColor, hoverAltitude):

        self.lineColor = lineColor
        self.platformColor = platformColor
        self.hoverAltitude = hoverAltitude
        self.processVideo = ProcessVideo()


    # Given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone isn't aligned vertically yet
    #   1 if algorithm is finished and drone is now aligned vertically
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        # color segmented images
        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        segPlatformImage = self.processVideo.DetectColor(image, self.platformColor)

        # trying to be parallel to the colored line while being over the platform
        angle = self.processVideo.ShowLine(segLineImage, thresh = 40)
        cx, cy = self.processVideo.CenterOfMass(segPlatformImage)

        xspeed, yspeed, zspeed = self.processVideo.ApproximateSpeed(segPlatformImage, cx, cy, 
        navdata["altitude"][1], self.hoverAltitude)

        yawspeed = self.processVideo.ObjectOrientation(segLineImage, cx, cy, angle)

        if xspeed == 0 and yspeed == 0 and zspeed == 0 and yawspeed == 0 and cx != None and cy != None:

            rospy.logwarn("Vertically facing " + self.lineColor + " line")
            directiveStatus = 1

        else:

            rospy.logwarn("Trying to vertically face " + self.lineColor + " line")
            directiveStatus = 0 

        return directiveStatus, (0.7*xspeed, 0.7*yspeed, yawspeed, zspeed), segLineImage




