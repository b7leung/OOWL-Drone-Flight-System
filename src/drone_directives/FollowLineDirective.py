#!/usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import cv2

# describes instruction on what the drone should do in order to 
# follow a line below it, given that the drone is perpendicular to the line
class FollowLineDirective(AbstractDroneDirective):
    
    # sets up this directive
    # lineColor: color of the line to follow
    def __init__(self, lineColor, speed = 0.4):

        self.lineColor = lineColor
        self.speed = speed
        self.processVideo = ProcessVideo()
        self.moveTime = 0.45
        self.waitTime = 0.1
        self.prevAngle = None
        self.prevAngleCount = 0
        self.prevAngleCountMax = 185


    # Given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone is still following line
    #   1 if algorithm is finished and has finished following line
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        platforms = navdata["allCenters"][1]
        lines, image = self.processVideo.MultiShowLine(segLineImage)

        if lines[0] != None:
            cv2.circle(image, lines[0][1], 15, (0,0,255), -1)
        if lines[1] != None:
            cv2.circle(image, lines[1][1], 15, (0,255,0), -1)
        if lines[2] != None:
            cv2.circle(image, lines[2][1], 15, (255,0,0), -1)

        linesVisible = (lines[0]!= None) + (lines[1] != None) + (lines[2] != None)

        if linesVisible == 0:
            rospy.logwarn(" *** ERROR: Lost " + self.lineColor + " line *** ")
            return -1, (0, 0, 0, 0), segLineImage, (None, None),0, 0, None

        cx = lines[1][1][0]
        cy = lines[1][1][1]
        _, yspeed, _ = self.processVideo.ApproximateSpeed(segLineImage, cx, cy, None,None, 
        navdata["SVCLAltitude"][1], 0, xtolerance = 80, ytolerance = 95)

        newCenter = None
        thresh = 15
        
        # Checking if angle is consistent with before
        if ( self.prevAngle == None or abs(self.prevAngle - lines[1][0]) < thresh 
        or abs(self.prevAngle - lines[1][0]) > (180 - thresh) ):

            self.prevAngle = lines[1][0]
            self.prevAngleCount = 0
            directiveStatus = 0

            # checking finish conditions
            # alternate way to finish
            xWindowSize = 200
            yWindowSize = 120
            xLower = 320 - xWindowSize
            yLower = 180 - yWindowSize
            xUpper = 320 + xWindowSize
            yUpper = 180 + yWindowSize

            foundRightPlatform = False
            tolerance = 15
            for platform in platforms:
                if( linesVisible > 1 and lines[1] != None and platform[0] > lines[1][1][0] and min(abs(lines[1][0] - 180), 180-abs(lines[1][0] - 180)) < 16 and
                platform[0] < xUpper and platform[0] > xLower and platform[1] < yUpper and platform[1] > yLower ):
                    cv2.rectangle(image, (xLower, yLower), (xUpper, yUpper), (255,255,255), 4)
                    foundRightPlatform = True
                    newCenter = platform

            # in order to be considered "finished", there must be 2 lines, 
            # one which is horizontal and one that is less than 90 degrees.
            # The horizontal line must be far enough left.
            if ( foundRightPlatform or (len(platforms) == 1 and yspeed == 0 and lines[1] != None and lines[2] != None and
            ( (lines[1][0] < (0 + tolerance) ) or (lines[1][0]) > (180-tolerance)) and
            lines[2][1][0] < int(640 * 0.9)) ):

                xspeed = 0
                yspeed = 0
                yawspeed = 0
                directiveStatus = 1
                rospy.logwarn("Finished following line")
                return directiveStatus, (xspeed, yspeed, yawspeed, 0), image, ((cx, cy), self.prevAngle), self.moveTime, self.waitTime, newCenter

        else:

            # only uses the angle if it is similar to the last one. If it is too different, algorithm
            # uses the previous angle for a time until the timer is up, as a buffer against a random large angle change

            rospy.logwarn("Large sudden change in angle -- using old angle of " 
            + str(self.prevAngle) + " instead of " + str(lines[1][0]))
            directiveStatus = -1
            
        xspeed = -self.speed

        # converting
        line1Angle = self.prevAngle
        if line1Angle == 90:
            line1Angle = 0
        elif line1Angle < 90:
            line1Angle = line1Angle + 90
        else:
            line1Angle = line1Angle - 90

        yawspeed = self.processVideo.LineOrientation(segLineImage, line1Angle, 8, yawspeed = 0.45)


        # If drone is still trying follow the line, it adapts to one of three algorithms:

        # Drone will just go back near the center if the drone is not "near" the
        # center as defined by a bounding box.
        # No turning or horizontal movement is applied.
        if yspeed != 0:
            rospy.logwarn("Moving blue line back to center")
            self.moveTime = 0.2
            self.waitTime = 0.1
            xspeed = 0
            yawspeed = 0

        # If drone is near the center but angle is off, it fixes the angle.
        # Drone does not move forward.
        elif yawspeed != 0:
            
            yawspeed = -yawspeed

            direction = "LEFT" 
            if yawspeed < 0:
                direction = "RIGHT"
                
            rospy.logwarn("Turning the drone horizontal " + direction + ",  yaw = " + str(yawspeed) )
            self.moveTime = 1
            self.waitTime = 0.1
            xspeed = 0

        else:
            rospy.logwarn("Drone just going forward")
            self.moveTime = 0.9
            self.waitTime = 0.1
                
        return directiveStatus, (xspeed, yspeed, yawspeed, 0), image, ((cx, cy), self.prevAngle), self.moveTime, self.waitTime, newCenter

    def Finished(self):
        self.prevAngle = None
        self.prevAngleCount = 0
        return None

