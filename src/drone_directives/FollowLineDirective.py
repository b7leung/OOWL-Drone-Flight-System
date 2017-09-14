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
        self.moveTime = 0
        self.waitTime = 0


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

        line1Angle, line1Center, line2Angle, line2Center = self.processVideo.ShowTwoLines(segLineImage)

        linesVisible = (line1Angle != None) + (line2Angle != None) 
        
        tolerance = 15

        cx = None
        cy = None

        
        if linesVisible == 0:
            rospy.logwarn(" *** ERROR: Lost " + self.lineColor + " line *** ")
            return -1, (0, 0, 0, 0), segLineImage, (None, None)

        elif ( ( (linesVisible == 1) and line1Angle < 90 and line1Angle > tolerance + 10 )
        or
        ( linesVisible == 2 and ( (line1Angle < (0 + tolerance)) or (180 -line1Angle) < tolerance ) and
        line2Angle + line1Angle < 90 + line1Angle and
        line1Center != None and line2Center != None and line1Center[0] < line2Center[0] ) ):

            xspeed = 0
            yspeed = 0
            yawspeed = 0
            directiveStatus = 1
            rospy.logwarn("Finished following line")
        
        else:
            
            directiveStatus = 0

            xspeed = -self.speed
            cx = line1Center[0]
            cy = line1Center[1]
            _, yspeed, _ = self.processVideo.ApproximateSpeed(segLineImage, cx, cy, 
            navdata["SVCLAltitude"][1], 0, xtolerance = 80, ytolerance = 80)

            if abs(yspeed) < 1:
                yspeed = yspeed *1.45
            
            # converting
            if line1Angle == 90:
                line1Angle = 0
            elif line1Angle < 90:
                line1Angle = line1Angle + 90
            else:
                line1Angle = line1Angle - 90

            yawspeed = self.processVideo.LineOrientation(segLineImage, line1Angle, 7)


            # If drone is still trying follow the line, it adapts to one of three algorithms:

            # Drone will just go back near the center if the drone is not "near" the
            # center as defined by a bounding box.
            # No turning or horizontal movement is applied.
            if yspeed != 0:
                rospy.logwarn("Moving blue line back to center")
                self.moveTime = 0.12
                self.waitTime = 0.01
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
                self.moveTime = 0.5
                self.waitTime = 0.01
                xspeed = 0

            else:
                rospy.logwarn("Drone just going forward")
                self.moveTime = 0.4
                self.waitTime = 0.01

        if line1Center != None:
            self.processVideo.DrawCircle(segLineImage,(line1Center[0],line1Center[1]))

        if line2Center != None:
            self.processVideo.DrawCircle(segLineImage,(line2Center[0],line2Center[1]))
                
        return directiveStatus, (xspeed, yspeed*1.1, yawspeed, 0), segLineImage, (cx, cy), self.moveTime, self.waitTime

