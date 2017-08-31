#!usr/bin/env python

import rospy
import cv2
from processing_functions.process_video import ProcessVideo
from processing_functions.pid_controller import PIDController
from AbstractDroneDirective import *

# describes instruction on what the drone should do in order
# to orient itself to a line underneath it
class PIDOrientLineDirective(AbstractDroneDirective):

    # orientation:
    #   > either "VERTICAL" or "PERPENDICULAR";
    #     algorithm will orient drone vertically or perpendicular to the line respectively
    # lineColor:
    #   > color of the line to orient to
    # platformColor:
    #   > color of the platform to orient to
    # settingsPath: 
    #   > Path for getting PID settings
    # hoverAltitude: 
    #   > how high to hover over the platform
    def __init__(self, orientation, lineColor, platformColor, settingsPath):

        if orientation != "PARALLEL" and orientation != "PERPENDICULAR":
            raise Exception("Orientation not recognized.")
        else:
            self.orientation = orientation
        self.lineColor = lineColor
        self.platformColor = platformColor
        self.settingsPath = settingsPath 

        self.processVideo = ProcessVideo()
        P,I,D = self.GetSettings()
        self.pid = PIDController()

    
    # reads PID settings 
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
    #   0 if algorithm is still running and drone isn't oriented yet
    #   1 if algorithm is finished and drone is now oriented
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        segLineImage = self.processVideo.DetectColor(image, self.lineColor)
        
        cx, cy = navdata["center"][1][0], navdata["center"][1][1]

        #draws center of circle on image
        self.processVideo.DrawCircle(segLineImage,(cx,cy))
        
        self.pid.UpdateDeltaTime()
        self.pid.SetPoint(image)
        self.pid.UpdateError(cx,cy)
        self.pid.SetPIDTerms()
        xspeed, yspeed = self.pid.GetPIDValues()

        if self.orientation == "PARALLEL":

            angle = self.processVideo.ShowLine(segLineImage,lowerAngleBound = 0, upperAngleBound = 70, secondBounds = (110,180), thresh = 35)
            yawspeed = self.processVideo.ObjectOrientation(segLineImage, angle, 5)
            xWindowSize = 50
            yWindowSize = 50


        elif self.orientation == "PERPENDICULAR":

            angle = self.processVideo.ShowLine(segLineImage, lowerAngleBound = 30, upperAngleBound = 125, thresh = 15)
            yawspeed = self.processVideo.LineOrientation(segLineImage, angle, 5)
            xWindowSize = 180
            yWindowSize = 70

        numRows, numCols, _ = image.shape
        centerx = numCols/2
        centery = numRows/2

        # box defines when the directive is finished
        xLower = centerx-xWindowSize
        yLower = centery-yWindowSize
        xUpper = centerx+xWindowSize
        yUpper = centery+yWindowSize
        cv2.rectangle(segLineImage, (xLower, yLower), (xUpper, yUpper), (255,255,255), 3)
        
        if ( yawspeed == 0
        and cx != None and cy != None and cx < xUpper and cx > xLower and cy < yUpper and cy > yLower):

            rospy.logwarn("Oriented " + self.orientation + " to " + self.lineColor + " line")
            directiveStatus = 1

        elif cx == None or cy == None:

            rospy.logwarn("*** ERROR: Lost " + self.platformColor + " platform ***")
            directiveStatus = -1

        else:


            # If drone is still trying to align, it adapts to one of three algorithms:
            
            # if this frame failed to detect a line, go towards platform
            # without turning in hopes that the next frames will detect one again
            if yawspeed == None:
                #rospy.logwarn("No line detected")
                yawspeed = 0

            # if drone is not "near" the center defined by a box, just focus on moving drone back;
            # no turning
            elif self.orientation == "PERPENDICULAR" and (cx > xUpper or cx < xLower or cy > yUpper or cy < yLower):
                cv2.rectangle(segLineImage, (xLower, yLower), (xUpper, yUpper), (0,0,255), 3)
                #rospy.logwarn("Only MOVING drone. x speed = " + str(xspeed) + "; y speed = " + str(yspeed))
                rospy.logwarn("Only MOVING drone")
                yawspeed = 0
                
            
            # if drone isn't perpendicular yet and is "near" the center (defined by a box),
            # just turn the drone; no need move drone
            elif yawspeed != 0:
                #rospy.logwarn("Only TURNING drone. yaw speed = " + str(yawspeed))
                rospy.logwarn("Only TURNING drone")
                xspeed = 0
                yspeed = 0

            else:

                rospy.logwarn("Only MOVING drone")
                #rospy.logwarn("Only MOVING drone. x speed = " + str(xspeed) + "; y speed = " + str(yspeed))
                
            directiveStatus = 0 
            
        return directiveStatus, (xspeed, yspeed, yawspeed, 0), segLineImage, (cx,cy)


    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        rospy.logwarn("***** finishing PID Orient Line Directive *****")
        self.pid.ResetPID()

