#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from processing_functions.pid_controller import PIDController
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order to orient itself
# perpendicular to a line underneath it
class PIDLineOrientDirective(AbstractDroneDirective):


    # sets up this directive
    # lineColor: color of the line to orient perpendicular to
    # platformColor: color of the platform to orient to
    # hoverAltitude: how high to hover over the platform
    def __init__(self, lineColor, platformColor):

        self.lineColor = lineColor
        self.platformColor = platformColor
        self.processVideo = ProcessVideo()
        self.pid = PIDController()


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
        numRows, numCols, channels = image.shape
        centerx = numCols/2
        centery = numRows/2
        windowSize = 40
        xLower = centerx-windowSize
        yLower = centery-windowSize
        xUpper = centerx+windowSize
        yUpper = centery+windowSize

        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        segPlatformImage = self.processVideo.DetectColor(image, self.platformColor)

        # trying to be perpendicular to the colored line while being over the platform
        angle = self.processVideo.ShowLine(segLineImage, lowerAngleBound = 30, upperAngleBound = 125, thresh = 15)
        #angle = self.processVideo.ShowLine(segLineImage, 20, 110, thresh = 45)
        cx, cy = self.processVideo.CenterOfMass(segPlatformImage)
        #draws center of circle on image
        self.processVideo.DrawCircle(segLineImage,(cx,cy))
        
        self.pid.UpdateDeltaTime()
        self.pid.SetPoint(segPlatformImage)
        self.pid.UpdateError(cx,cy)
        self.pid.SetPIDTerms()
        xspeed, yspeed = self.pid.GetPIDValues()
        self.pid.DrawArrow(segPlatformImage, xspeed, yspeed)
        yawspeed = self.processVideo.LineOrientation(segLineImage, angle, 5)

        if ( xspeed == 0 and yspeed == 0 and yawspeed == 0
        and cx != None and cy != None and cx < xUpper and cx > xLower and cy < yUpper and cy > yLower):

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
                
            directiveStatus = 0 
            rospy.logwarn("Trying to align perpendicularly to " + self.lineColor + " line")
            
        return directiveStatus, (xspeed, yspeed, yawspeed, 0), segLineImage, (cx,cy)


