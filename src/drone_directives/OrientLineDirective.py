#!usr/bin/env python

import rospy
import cv2
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import math

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
        self.prevCenter = None


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
        
        if cx != None and cy != None:
            cv2.circle(segLineImage, (cx,cy), 6, (255,255,255), -1)

        # when directive first starts, it latches onto the first orange platform it sees
        if self.prevCenter == None:

            if cx != None and cy != None:
                self.prevCenter = (cx,cy)

        elif cx != None and cy != None:

            # checking if curr center is consistent with previous one
            centerDist = math.sqrt( math.pow((self.prevCenter[1] - cy),2) 
            + math.pow((self.prevCenter[0] - cx),2 ) ) 
            if centerDist > 170:
                rospy.logwarn("ERROR: ORIGINAL CENTER LOST")
                cx = self.prevCenter[0]
                cy = self.prevCenter[1]
                directiveStatus = -1
                return directiveStatus, (0,0,0,0), segLineImage, (cx,cy), 0,0
            else:
                self.prevCenter = (cx,cy)

        lines, segLineImage = self.processVideo.MultiShowLine(segLineImage)

        if self.orientation == "PARALLEL":
            
            # pick the green line closest to the hover platform
            angle = None
            closest = None
            for line in lines:
                if( line != None and cx!= None and
                (closest == None or abs( cx - line[1][0] ) < abs( cx - closest[1][0])) ):

                    closest = line
                    angle = closest[0]
            
            if closest != None: 
                cv2.circle(segLineImage, closest[1], 15, (0,255,0), -1)
                for line in lines:
                    if line!= None and line[1] != closest[1]:
                        cv2.circle(segLineImage, line[1], 15, (0,0,255), -1)

            #converting angle
            if angle != None:
                if angle == 90:
                    angle = 0
                elif angle < 90:
                    angle = angle + 90
                else:
                    angle = angle - 90

            yawspeed = self.processVideo.ObjectOrientation(segLineImage, angle, 8, yawspeed = 0.45)
            if yawspeed!=None:
                yawspeed = -1*yawspeed
            xWindowSize = 60
            yWindowSize = 60
            altLowerTolerance = 120
            altUpperTolerance = 120

        elif self.orientation == "PERPENDICULAR":
            
            # pick the blue line closest to the hover platform, AND is right of the hover platform
            angle = None
            closest = None
            for line in lines:
                if( line != None and cx != None and line[1][0] >= cx and
                (  closest == None or  abs( cx - line[1][0] ) < abs( cx - closest[1][0]) ) ):
                    closest = line
                    angle = closest[0]

            if closest != None: 
                cv2.circle(segLineImage, closest[1], 15, (0,255,0), -1)
                for line in lines:
                    if line!= None and line[1] != closest[1]:
                        cv2.circle(segLineImage, line[1], 15, (0,0,255), -1)

            #converting angle
            if angle != None:
                if angle == 90:
                    angle = 0
                elif angle < 90:
                    angle = angle + 90
                else:
                    angle = angle - 90

            yawspeed = self.processVideo.LineOrientation(segLineImage, angle, 12, yawspeed = 0.4)
            if yawspeed!=None:
                yawspeed = -1*yawspeed
            xWindowSize = 185
            yWindowSize = 65
            altLowerTolerance = 200
            altUpperTolerance = 300
        
        # defines window to make the drone focus on moving away from the edges and back into
        # the center; yaw will be turned off
        xReturnSize = 180
        yReturnSize = 70
        
        xspeed, yspeed, zspeed = self.processVideo.ApproximateSpeed(segLineImage, cx, cy, 
        navdata["SVCLAltitude"][1], self.hoverAltitude, 
        xtolerance = xWindowSize, ytolerance = yWindowSize, ztolerance = (altLowerTolerance, altUpperTolerance))

        #draws center of circle on image
        
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


    def Finished(self):
        self.prevCenter = None

