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
    def __init__(self, lineColor, speed = 0.1):

        self.lineColor = lineColor
        self.speed = speed
        self.processVideo = ProcessVideo()


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

        height, _, _ = image.shape
        
        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        lines = self.processVideo.GetLines(segLineImage, thresh = 30)

        angle = self.processVideo.ProcessLines(lines, segLineImage, lowerAngleBound = 80, upperAngleBound = 100, lineColor = (0, 0, 255))
        #correctingAngle = self.processVideo.ProcessLines(lines, segLineImage, lowerAngleBound = 30, upperAngleBound = 150, lineWidth = 1 )
        
        yawspeed = self.processVideo.LineOrientation(segLineImage, angle, 5)

        cx, cy = self.processVideo.CenterOfMass(segLineImage)

        _, yspeed, _ = self.processVideo.ApproximateSpeed(segLineImage, cx, cy, 
        navdata["SVCLAltitude"][1], 0, xtolerance = 80, ytolerance = 80)

        if abs(yspeed) < 1:
            yspeed = yspeed *1.45

        numRows, numCols, _ = image.shape
        centerx = numCols/2
        centery = numRows/2

        if angle == None and (cx != None or cx < centerx):

            xspeed = 0
            yspeed = 0
            yawspeed = 0
            directiveStatus = 1
            rospy.logwarn("Finished following line")
            
        else:

            xspeed = -self.speed
            directiveStatus = 0

            # If drone is still trying follow the line, it adapts to one of three algorithms:

            # Drone will just go back near the center if the drone is not "near" the
            # center as defined by a bounding box.
            # No turning or horizontal movement is applied.
            if yspeed != 0:
                rospy.logwarn("Moving blue line back to center")
                xspeed = 0
                yawspeed = 0

            # If drone is near the center but angle is off, it fixes the angle.
            # Drone does not move forward.
            elif yawspeed != 0:
                rospy.logwarn("Turning the drone horizontal")
                xspeed = 0

            else:
                rospy.logwarn("Drone just going forward")

        return directiveStatus, (xspeed, yspeed*1.1, yawspeed, 0), segLineImage, (None, None)



