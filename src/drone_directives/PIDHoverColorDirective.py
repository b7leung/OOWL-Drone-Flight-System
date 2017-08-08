#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
from processing_functions.pid_controller import PIDController

# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class PIDHoverColorDirective(AbstractDroneDirective):
    
    # sets up this directive
    # platformColor: color to hover over
    def __init__(self, platformColor):

        self.platformColor = platformColor 
        self.processVideo = ProcessVideo()
        self.pid = PIDController()
        
    

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
        
        orange_image = self.processVideo.DetectColor(image, self.platformColor)
        cx, cy = self.processVideo.CenterOfMass(orange_image)
        self.pid.UpdateDeltaTime()
        self.pid.SetPoint(orange_image)
        self.pid.SetPIDConstants(0.4, 0.0, 0.0)
        self.pid.UpdateError(cx,cy)
        x_P, y_P, x_I, y_I, x_D, y_D = self.pid.SetPIDTerms()
        xspeed, yspeed = self.pid.GetPIDValues()

        #rospy.logwarn("X Terms: "+str(x_P)+", " + str(x_I)+", "+ str(x_D))
        #rospy.logwarn("Y Terms: "+str(y_P)+", " + str(y_I)+", "+ str(y_D))

        rospy.logwarn("xPID, yPID: "+str(xspeed) +", " + str(yspeed))
        #self.MoveFixedTime(xspeed, yspeed, 0 ,0, 0.1, 0.01)

        # if there is orange in the screen, and the drone is in the middle, return true
        if cx != None and cy != None and xspeed == 0 and yspeed == 0:

            rospy.logwarn("PID: Done Hovering on " + self.platformColor)
            directiveStatus = 1

        elif cx == None or cy == None:
            
            directiveStatus = -1

        else:

            rospy.logwarn("PID: Trying to Hover on " + self.platformColor)
            directiveStatus = 0

        return directiveStatus, (xspeed, yspeed, 0, 0), orange_image, (cx,cy)




        
