#!usr/bin/env python

import rospy
import cv2
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
from processing_functions.pid_controller import PIDController
from os.path import expanduser
from std_msgs.msg import Int32, Float32

# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class PIDHoverColorDirective2(AbstractDroneDirective):
    
    # sets up this directive
    # platformColor: color to hover over. Altitude is maintained
    def __init__(self, platformColor):
        
        self.platformColor = platformColor 
        self.processVideo = ProcessVideo()
        P,I,D = 0.0054, 0.006352, 0.0011475
        #P,I,D = 0.00405, 0.00285, 0
        self.pid = PIDController(Kp = P, Ki = I, Kd = D)
        self.moveTime = 0.2
        self.waitTime = 0
        self.pub_pid_xspeed = rospy.Publisher('pid_xspeed', Float32, queue_size = 10)
        self.pub_pid_yspeed = rospy.Publisher('pid_yspeed', Float32, queue_size = 10)
        self.pub_pid_in_alt = rospy.Publisher('pid_in_alt', Int32, queue_size = 10)
        rate = rospy.Rate(5)


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
        altitude = navdata["altitude"][1]
        self.pub_pid_in_alt.publish(altitude)

        if cx != None and cy != None:
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 
        numRows, numCols, _ = image.shape
        centerx = numCols/2
        centery = numRows/2
        windowSize = 40
        xLower = centerx-windowSize
        yLower = centery-windowSize
        xUpper = centerx+windowSize
        yUpper = centery+windowSize  
        cv2.rectangle(image, (xLower, yLower), (xUpper, yUpper), (255,255,255), 3)
        
        xspeed, yspeed = self.pid.getOutput(cx,cy,altitude)

        #self.pid.UpdateDeltaTime()
        #self.pid.UpdateError(cx,cy,altitude)
        #self.pid.SetPIDTerms()

        #xspeed, yspeed = self.pid.GetPIDValues()
        self.pub_pid_xspeed.publish(xspeed)
        self.pub_pid_yspeed.publish(yspeed)

        yspeed = 0 

        rospy.logwarn("Xspeed = " + str(xspeed) +" Yspeed = " + str(yspeed))
        
        directiveStatus = 1
   
        return directiveStatus, (xspeed, yspeed, 0, 0), image, (cx,cy), self.moveTime, self.waitTime, None


    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        self.Reset()

    def Reset(self):
        self.pid.ResetPID()
        



