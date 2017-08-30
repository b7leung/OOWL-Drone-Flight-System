#!usr/bin/env python

import rospy
import cv2
from processing_functions.process_video import ProcessVideo
from processing_functions.pid_controller import PIDController
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order to orient itself
# vertically with a line underneath it
class PIDObjectOrientDirective(AbstractDroneDirective):


    # sets up this directive
    # lineColor: color of the line to orient vertically with
    # platformColor: color of the platform to orient with
    # hoverAltitude: how high to hover over the platform
    def __init__(self, lineColor, platformColor, settingsPath):

        self.lineColor = lineColor
        self.platformColor = platformColor
        self.processVideo = ProcessVideo()
        self.settingsPath = settingsPath
        P, I, D = self.GetSettings()
        self.pid = PIDController(P, I, D)


    def GetSettings(self):
        # read a text file as a list of lines
        # find the last line, change to a file you have
        fileHandle = open ( self.settingsPath,'r' )
        last = fileHandle.readlines()
        fileHandle.close()        
        
        last=str(last[len(last)-1]).split()
        #rospy.logwarn(str(last))
        p, i, d = [float(x) for x in (last)]
        
        return p, i ,d


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

        
        segLineImage = self.processVideo.DetectColor(image, self.lineColor)

        # trying to be parallel to the colored line while being over the platform
        angle = self.processVideo.ShowLine(segLineImage,lowerAngleBound = 0, upperAngleBound = 70, secondBounds = (110,180), thresh = 35)

        cx, cy = navdata["center"][1][0], navdata["center"][1][1]

        #Draws a circle over the center of platform on lined image
        self.processVideo.DrawCircle(segLineImage,(cx,cy))

        self.pid.UpdateDeltaTime()
        self.pid.SetPoint(image)
        self.pid.UpdateError(cx,cy)
        self.pid.SetPIDTerms()
        xspeed, yspeed = self.pid.GetPIDValues()

        yawspeed = self.processVideo.ObjectOrientation(segLineImage, angle, 5)

        # Directive is finished if drone is within a box of the center
        numRows, numCols, channels = image.shape
        centerx = numCols/2
        centery = numRows/2
        windowSize = 50

        xLower = centerx-windowSize
        yLower = centery-windowSize
        xUpper = centerx+windowSize
        yUpper = centery+windowSize

        cv2.rectangle(segLineImage, (xLower, yLower), (xUpper, yUpper), (255,255,255), 3)

        if ( yawspeed == 0
        and cx != None and cy != None and cx < xUpper and cx > xLower and cy < yUpper and cy > yLower):

            rospy.logwarn("PID: Vertically facing " + self.lineColor + " line")
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

            # if a line was found and drone isn't vertical yet,
            # just turn the drone; no need move drone
            elif yawspeed != 0:

                xspeed = 0
                yspeed = 0

                if yawspeed < 0:
                    pass
                else:
                    xspeed = 0
                    yspeed = 0

            directiveStatus = 0 
            rospy.logwarn("PID: Trying to vertically face " + self.lineColor + " line")
        #rospy.logwarn("running PID object orient")
        #return 0, (0, 0, 0, 0), segLineImage, (0,0)
        return directiveStatus, (xspeed, yspeed, yawspeed, 0), segLineImage, (cx,cy)




