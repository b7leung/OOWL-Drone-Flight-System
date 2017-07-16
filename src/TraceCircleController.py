#!/usr/bin/env python

import rospy
import time
import datetime

from os.path import expanduser

from drone_video import DroneVideo
from drone_controller import BasicDroneController
from processing_functions.process_video import ProcessVideo
from processing_functions.process_position import DronePosition


import cv2

class TraceCircleController(DroneVideo):

    def __init__(self):


        # Call DroneVideo's constructor so we are subscribed to video feed 
        # will be converted to CVImage and edited by EditVideo() before shown.
        super(TraceCircleController,self).__init__()
        
        #set up helper objects
        self.process = ProcessVideo()
        self.controller = BasicDroneController()

        self.startControl = False
        self.startTimer = time.clock()

        # clears any previous log file that might exist, and sets up a new log file
        self.startTime = time.clock()
        self.logFilePath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/output.txt"
        open(self.logFilePath, "w").close()
        self.logFile=open( self.logFilePath,"a")
        datetime.datetime.now().date
        self.logFile.write( " ===== " + "Log created as of " + 
            datetime.datetime.now().strftime("%A, %B %d %Y: %I:%M%p") + " ===== " +"\n" )


    # define any keys to listen to here
    def KeyListener(self):
        
        #waits for a keypress indefinitly
        key=cv2.waitKey(1) & 0xFF

        if key == ord('i'):
            if self.startControl:
                self.startControl= False
            else:
                self.startControl = True


    # overriding superclass's EditVideo method
    # can change self.cv_image here, and changes will be reflected on the video
    def EditVideo(self):

        if self.startControl:
            self.hoverOnOrange()
        
        #self.cv_image = self.process.ShowLine(self.cv_image)


    def hoverOnOrange(self):
        
        # calculating cx,cy,xspeed,yspeed
        orange_image=self.process.DetectColor(self.cv_image,'orange')
        self.cv_image=orange_image
        cx,cy=self.process.CenterofMass(orange_image)
        xspeed,yspeed=self.process.ApproximateSpeed(orange_image,cx,cy)
        
        # logging information; timeElapsed is in milliseconds
        timeElapsed = (time.clock()-self.startTime)*1000
        currentDroneInfo = ("time: " +str(timeElapsed)+ " cx: " + str(cx) +
        " cy: " + str(cy) + " xspeed: " + str(xspeed) + " yspeed: " + str(yspeed) + "\n")
        self.logFile.write(currentDroneInfo)

        # move drone corresponding to xspeed and yspeed at a fixed interval
        self.MoveFixedTime(xspeed,yspeed,0.25,0.15)
        #self.MoveFixedTime(xspeed,yspeed,0.25,0.3)
            
    
   #this function will go a certain speed for a set amount of time
    def MoveFixedTime(self,xspeed,yspeed,move_time,wait_time):
        current_time=time.clock()

        if current_time > (self.startTimer+move_time):
            self.controller.SetCommand(0,0)
        if current_time > (self.startTimer+move_time+wait_time):
            self.startTimer=time.clock()
            self.controller.SetCommand(xspeed,yspeed)
        if current_time > (2*(self.startTimer+move_time+wait_time)):
            rospy.logwarn("third")
            self.controller.SetCommand(0,0)
            self.startTimer=time.clock()


    # if 0.2 % of what the drone sees is orange, then it will go forward
    # orange corresponds to a hueMin of 0 and a hueMax of 50
    def goForwardIfOrange(self):
        orangeVisible = self.process.isHueDominant(self.cv_image, 0, 50, 0.2); 
        if orangeVisible:
            rospy.logwarn("go forward")
            self.controller.SetCommand(pitch = 0.03)
        else:
            rospy.logwarn("stop")
            self.controller.SetCommand(pitch = 0)

if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.spin()

    

