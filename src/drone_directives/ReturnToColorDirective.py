#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
import numpy
import math
import cv2

# This directive will give directions that attempt to push the drone
# back towards the target location if it has drifted out of the cameras vision
class ReturnToColorDirective(AbstractDroneDirective):


    # sets up this directive
    # platformColor: color of the platform to return to
    def __init__(self, platformColor, lineColor, speedModifier = 0.6, radiusThresh = 170):

        self.platformColor = platformColor
        self.processVideo = ProcessVideo()
        self.speedModifier = speedModifier
        self.radiusThresh = radiusThresh
        self.lineColor = lineColor
        self.moveTime = 0.35
        self.waitTime = 0.10
        self.bestPlatformFound = None
    
    def InsideCircle(self, point, circleCenter, circleRadius):
        x = point[0]
        y = point[1]
        center_x = circleCenter[0]
        center_y = circleCenter[1]
        radius = circleRadius
        
        return (math.pow((x-center_x),2) + math.pow((y-center_y),2)) < math.pow(radius,2)

    def PointAlongLine(self, linePoint, lineAngle, point, thresh):
        # edge case: both lines are vertical
        if point[0] == linePoint[0]:
            if lineAngle == 90:
                return True
            else:
                return False
        else:
            # slope compensates for a upper left origin coord system
            slope = (float(linePoint[1])-point[1]) / (point[0]-float(linePoint[0]))

            slopeAngle = math.degrees(math.atan(slope))
            if slopeAngle < 0:
                #rospy.logwarn("seen added 180")
                slopeAngle += 180
            #rospy.logwarn(str(point) + str(linePoint) +" >> slope: " + str(slope))
            #rospy.logwarn("Original: " + str(lineAngle) + " seen: " + str(slopeAngle))
            if( min( abs( lineAngle - slopeAngle), 180 - abs( lineAngle - slopeAngle) ) < thresh):
                #rospy.logwarn("GOOD")
                return True
            else:
                #rospy.logwarn("BAD")
                return False

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
        
        lineSeg = self.processVideo.DetectColor(image, self.lineColor)
        #platformSeg = self.processVideo.DetectColor(image, self.platformColor)
        #platformSeg = self.processVideo.RemoveNoise(platformSeg)
        #platformSeg = navdata[0]["segImage"]
        #centers, _ = self.processVideo.MultiCenterOfMass(platformSeg)
        centers = navdata[0]["allCenters"][1]

        #navdata stores the last location in the case of an error
        cx = navdata[1][0][0]
        cy = navdata[1][0][1] 
        lastAngle = navdata[1][1]
        
        hasPlatform = False

        # find last platform based on last seen line angle
        if lastAngle != None:
            lines, image = self.processVideo.MultiShowLine(lineSeg, sort = False)
            for center in centers:
                cv2.circle(image, center, 4, (255,255,255), -1)

            thresh = 21
            validLine = None
            # picks the line closest to last angle, and within thresh (degrees)
            for line in lines:
                #rospy.logwarn("last: " + str(lastAngle) + " this: " + str(line[0]))
                angleDifference = min( abs(lastAngle -line[0]), 180 -abs(lastAngle -line[0]) ) 
                if( line != None and angleDifference < thresh and (validLine == None or validLine[1] > angleDifference) ):
                    #rospy.logwarn("valid")
                    validLine = (line, angleDifference)

            if validLine != None:
                line = validLine[0]
                # finding center closest to the most valid line
                for c in centers:
                    alongLine = self.PointAlongLine(line[1], line[0], c, 25)
                    # blue line => orient perpendicular => valid point must be to the left of line
                    if self.lineColor == "blue" and c[0] > line[1][0]:
                        alongLine = False
                    if alongLine:
                        lastAngle = line[0]
                        cv2.line(image, line[1], c, (0,255,255),3)
                        cx, cy = c[0], c[1]
                        cv2.circle(image, (cx,cy), 12, (0,255,0), -1)
                        cv2.circle(image, (cx,cy), 12, (255,255,255), 7)
                        cv2.circle(image, line[1], 7, (0,255,0), -1)
                        cv2.circle(image, line[1], 7, (255,255,255), 4)
                        hasPlatform = True

        # if no angle was found, just use location
        else:

            image = navdata[0]["segImage"]

            cv2.circle(image, (cx,cy), self.radiusThresh, (0,255,0), 1)
            cv2.circle(image, (cx,cy), 7, (0,255,0), -1)

            for c in centers:

                cv2.circle(image, c, 10, (0,255,255), -1)
                if self.InsideCircle( c , (cx,cy), self.radiusThresh):
                    hasPlatform = True
                    cx, cy = c[0], c[1]
        
        if hasPlatform:
            rospy.logwarn("Successfully returned to platform -- last angle seen was "+ str(lastAngle))
            directiveStatus = 1
            zspeed = 0

        else:
            rospy.logwarn("Returning to platform -- last angle seen was "+ str(lastAngle))
            directiveStatus = 0
            zspeed = 0.2

        xspeed, yspeed, _ = self.processVideo.ApproximateSpeed(image.copy(), cx, cy,
        ytolerance = 50, xtolerance = 50)
        
        xspeed = min( xspeed * self.speedModifier, 1 )
        yspeed = min( yspeed * self.speedModifier, 1 )
        rospy.logwarn("X Speed: " + str(xspeed) + " Y Speed: " + str(yspeed))
        
        # draw rectangles so it's easy to tell that it's in return mode
        border = 15
        offset = 2
        cv2.rectangle(image, (border, border), (640-border,360-border), (0,0, 255), 1)
        cv2.rectangle(image, (border-1*offset, border-1*offset), (640-border+1*offset,360-border+1*offset), (0,229, 255), 1)
        cv2.rectangle(image, (border-2*offset, border-2*offset), (640-border+2*offset,360-border+2*offset), (0,0, 255), 1)

        return directiveStatus, (xspeed, yspeed, 0, zspeed), image, ((cx,cy), lastAngle), self.moveTime, self.waitTime, None

    def Finished(self):
        rospy.logwarn("RETURN TO COLOR FINISHED *******************")
        rospy.logwarn("RETURN TO COLOR FINISHED *******************")
        self.bestPlatformFound = None
        return None
        

