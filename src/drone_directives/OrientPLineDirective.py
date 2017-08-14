#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order to orient itself
# perpendicular to a line underneath it
class OrientPLineDirective(AbstractDroneDirective):


    # sets up this directive
    # lineColor: color of the line to orient perpendicular to
    # platformColor: color of the platform to orient to
    # hoverAltitude: how high to hover over the platform
    def __init__(self, lineColor, platformColor, hoverAltitude):

        self.lineColor = lineColor
        self.platformColor = platformColor
        self.hoverAltitude = hoverAltitude
        self.processVideo = ProcessVideo()


    # Given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone isn't perpendicular yet
    #   1 if algorithm is finished and drone is now perpendicular
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        # color segmented images
        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        segPlatformImage = self.processVideo.DetectColor(image, self.platformColor)

        # trying to be perpendicular to the colored line while being over the platform
        angle = self.processVideo.ShowLine(segLineImage, thresh = 20)
        #angle = self.processVideo.ShowLine(segLineImage, 20, 110, thresh = 45)
        cx, cy = self.processVideo.CenterOfMass(segPlatformImage)
        #draws center of circle on image
        self.processVideo.DrawCircle(segLineImage,(cx,cy))
        
        xspeed, yspeed, zspeed = self.processVideo.ApproximateSpeed(segPlatformImage, cx, cy, 
        navdata["altitude"][1], self.hoverAltitude, tolerance = 20)

        yawspeed = self.processVideo.LineOrientation(segLineImage, angle, 10)

        if ( xspeed == 0 and yspeed == 0 and zspeed == 0 and yawspeed == 0
        and cx != None and cy != None ):

            rospy.logwarn("Perpendicular to " + self.lineColor + " line")
            directiveStatus = 1

        elif cx == None or cy == None:

            rospy.logwarn("*** ERROR: Lost " + self.platformColor + " platform ***")
            directiveStatus = -1

        else:
            
            # If drone is still trying to align, it adapts to one of two algorithms:

            # if this frame failed to detect a line, go towards platform
            # without turning in hopes that the next frames will detect one again
            if yawspeed == None:
                yawspeed = 0

            # if a line was found and drone isn't perpendicular yet,
            # just turn the drone; no need move drone
            elif yawspeed != 0:
                xspeed = 0
                yspeed = 0
                zspeed = 0
                
            directiveStatus = 0 
            rospy.logwarn("Trying to align perpendicularly to " + self.lineColor + " line")
            
        return directiveStatus, (xspeed, yspeed, yawspeed, zspeed), segLineImage, (cx,cy)


