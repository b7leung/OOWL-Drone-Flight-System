#!usr/bin/env python

import rospy
import cv2
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
from processing_functions.pid_controller import PIDController
from os.path import expanduser

# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class PIDHoverColorDirective(AbstractDroneDirective):
    
    # sets up this directive
    # platformColor: color to hover over
    # settingsPath: path of file to read PID settings from
    def __init__(self, platformColor,settingsPath):
        
        self.platformColor = platformColor 
        self.processVideo = ProcessVideo()
        P,I,D = self.GetSettings(settingsPath)
        self.pid = PIDController(360, 640, Kp = P, Ki = I, Kd = D)
        
        self.moveTime = 0.2
        self.waitTime = 0.1


    def GetSettings(self, settingsPath):
        # read a text file as a list of lines
        # find the last line, change to a file you have
        fileHandle = open ( settingsPath,'r' )
        last = fileHandle.readlines()
        fileHandle.close()        
        
        last=str(last[len(last)-1]).split()
        p, i, d = [float(x) for x in (last)]
        
        return p, i ,d


    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone isn't on orange yet
    #   1 if algorithm is finished and drone is now on orange
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        image = navdata["segImage"]
                            
        cx, cy = navdata["center"][1][0], navdata["center"][1][1]
        altitude = navdata["SVCLAltitude"][1]
        if cx != None and cy != None:
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 

        self.pid.UpdateDeltaTime()
        self.pid.UpdateError(cx,cy,altitude)
        self.pid.SetPIDTerms()
        xspeed, yspeed = self.pid.GetPIDValues()



        numRows, numCols, _ = image.shape
        centerx = numCols/2
        centery = numRows/2
        windowSize = 40
        xLower = centerx-windowSize
        yLower = centery-windowSize
        xUpper = centerx+windowSize
        yUpper = centery+windowSize
       
        self.pid.DrawArrow(image, xspeed, yspeed)
        cv2.rectangle(image, (xLower, yLower), (xUpper, yUpper), (255,255,255), 3)

        # if there is orange in the screen, and the drone is in the middle, return true
        if cx != None and cy != None and cx < xUpper and cx > xLower and cy < yUpper and cy > yLower:

            '''rospy.logwarn("PID: Done Hovering on " + self.platformColor)
            '''
            directiveStatus = 1


        elif cx == None or cy == None:
            
            '''rospy.logwarn("")
            rospy.logwarn("PID: ERROR -- couldn't find " + self.platformColor + " platform")
            '''
            directiveStatus = -1

        else:
            p,i,d = self.pid.ReturnPIDvalues()
            directiveStatus = 0
        
        return directiveStatus, (xspeed, yspeed, 0, 0), image, (cx,cy), self.moveTime, self.waitTime, None


    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        rospy.logwarn("***** Resetting PID Values *****")
        self.pid.ResetPID()
        


