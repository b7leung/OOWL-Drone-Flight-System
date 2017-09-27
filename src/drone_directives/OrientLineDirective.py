#!usr/bin/env python

import rospy
import cv2
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order to orient itself
# to a line underneath it
class OrientLineDirective(AbstractDroneDirective):

    # orientation:
    #   > either "VERTICAL" or "PERPENDICULAR";
    #     algorithm will orient drone vertically or perpendicular to the line respectively
    # lineColor:
    #   > color of the line to orient to
    # platformColor:
    #   > color of the platform to orient to
    # hoverAltitude: 
    #   > how high to hover over the platform
    def __init__(self, orientation, lineColor, platformColor, hoverAltitude):

        if orientation != "PARALLEL" and orientation != "PERPENDICULAR":
            raise Exception("Orientation not recognized.")
        else:
            self.orientation = orientation

        self.lineColor = lineColor
        self.platformColor = platformColor
        self.hoverAltitude = hoverAltitude
        self.processVideo = ProcessVideo()
        self.moveTime=0.2
        self.waitTime=0.1

    # Given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone isn't oriented yet
    #   1 if algorithm is finished and drone is now oriented
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        self.moveTime = 0.2

        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        
        cx, cy = navdata["center"][1][0], navdata["center"][1][1]
        
        if self.orientation == "PARALLEL":

            angle = self.processVideo.ShowLine(segLineImage,lowerAngleBound = 0, upperAngleBound = 70, secondBounds = (110,180), thresh = 35)
            yawspeed = self.processVideo.ObjectOrientation(segLineImage, angle, 4, yawspeed = 0.55)
            xWindowSize = 60
            yWindowSize = 60
            altLowerTolerance = 120
            altUpperTolerance = 120

        elif self.orientation == "PERPENDICULAR":

            angle = self.processVideo.ShowLine(segLineImage, lowerAngleBound = 45, upperAngleBound = 110, thresh = 15)
            yawspeed = self.processVideo.LineOrientation(segLineImage, angle, 4, yawspeed = 0.55)
            xWindowSize = 185
            yWindowSize = 65
            altLowerTolerance = 500
            altUpperTolerance = 150
        
        # defines window to make the drone focus on moving away from the edges and back into
        # the center; yaw will be turned off
        xReturnSize = 180
        yReturnSize = 70
        
        xspeed, yspeed, zspeed = self.processVideo.ApproximateSpeed(segLineImage, cx, cy, 
        navdata["SVCLAltitude"][1], self.hoverAltitude, 
        xtolerance = xWindowSize, ytolerance = yWindowSize, ztolerance = (altLowerTolerance, altUpperTolerance))

        #draws center of circle on image
        self.processVideo.DrawCircle(segLineImage,(cx,cy))
        
        numRows, numCols, _ = image.shape
        centerx = numCols/2
        centery = numRows/2

        # box defines when the directive is finished
        xLower = centerx-xReturnSize
        yLower = centery-yReturnSize
        xUpper = centerx+xReturnSize
        yUpper = centery+yReturnSize

        # perpendicular can disregard height
        if self.orientation == "PERPENDICULAR":
            zspeed = 0

        if ( yawspeed == 0 and xspeed == 0 and yspeed == 0 and zspeed == 0 and cx != None and cy != None ):
            
            # Double check
            xLowerC = centerx-xWindowSize
            yLowerC = centery-yWindowSize
            xUpperC = centerx+xWindowSize
            yUpperC = centery+yWindowSize
            if ( cx >= xUpperC or cx <= xLowerC or cy >= yUpperC or cy <= yLowerC ):
                rospy.logwarn("FALSE POSITIVE")
                directiveStatus = 0
            else:
                rospy.logwarn("Oriented " + self.orientation + " to " + self.lineColor + " line")
                directiveStatus = 1

        elif cx == None or cy == None:

            rospy.logwarn("*** ERROR: Lost " + self.platformColor + " platform ***")
            directiveStatus = -1

        else:

            # If drone is still trying to align, it adapts to one of three algorithms:
            
            # Drone will just go back near the center if: 1) no line is detcted, or 2)
            # the drone is not "near" the center as defined by a bounding box
            # No turning or altitude change applied
            if yawspeed == None or ( cx > xUpper or cx < xLower or cy > yUpper or cy < yLower ):
                cv2.rectangle(segLineImage, (xLower, yLower), (xUpper, yUpper), (0,0,255), 2)
                rospy.logwarn("Too far out; only MOVING drone back to center")
                yawspeed = 0
                zspeed = 0

            # if drone isn't perpendicular yet and is "near" the center (defined by a box),
            # just turn the drone; no need move drone
            elif yawspeed != 0:
                rospy.logwarn("Only TURNING drone. Yaw speed = " + str(yawspeed))
                self.moveTime = 3.5
                xspeed = 0
                yspeed = 0
                zspeed = 0
            
            # if the drone is aligned to the line and is near the center, 
            # keep moving it to the center and adjusting the height until the 
            # directive is finished
            else:
                rospy.logwarn("Curr Altitude = " + str( int(navdata["SVCLAltitude"][1])) +
                " mm; Goal = [ " + str(self.hoverAltitude - altLowerTolerance) + " mm, " + 
                str(self.hoverAltitude + altUpperTolerance) + " mm ].")
                
            directiveStatus = 0 

        return directiveStatus, (xspeed, yspeed, yawspeed, zspeed), segLineImage, (cx,cy), self.moveTime, self.waitTime
