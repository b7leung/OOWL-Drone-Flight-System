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
        self.controller = BasicDroneController("TraceCircle")

        self.startControl = False
        self.startTimer = time.clock()
        self.state = 'IDLE'
        
        # clears any previous log file that might exist, and sets up a new log file
        self.startTime = time.clock()
        self.logFilePath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/output.txt"
        open(self.logFilePath, "w").close()
        self.logFile=open( self.logFilePath,"a")
        datetime.datetime.now().date
        self.logFile.write( " ===== " + "Log created for flight on " + 
            datetime.datetime.now().strftime("%A, %B %d %Y: %I:%M%p") + " ===== " +"\n" )


    # define any keys to listen to here
    def KeyListener(self):
        
        #waits for a keypress 
        key=cv2.waitKey(1) & 0xFF

        if key == ord('i'):
            if self.startControl:
                # if toggle was on, toggle it off
                self.startControl= False
                # make the drone just hover in place
                self.controller.SetCommand(0,0,0,0)
                self.state='IDLE'
            else:
                # if toggle was off, toggle it on
                #self.startTimer=time.clock()
                self.startControl = True
                self.state = 'HoverOnOrange'
        
        elif key == ord('b'):
            if self.startControl:
                # if toggle was on, toggle it off
                self.startControl= False
                # make the drone just hover in place
                self.controller.SetCommand(0,0,0,0)
                self.state='IDLE'
            else:
                # if toggle was off, toggle it on
                #self.startTimer=time.clock()
                self.startControl = True
                self.state = 'FollowBlue'

        elif key == ord('m'):
            if self.startControl:
                # if toggle was on, toggle it off
                self.startControl= False
                # make the drone just hover in place
                self.controller.SetCommand(0,0,0,0)
                self.state='IDLE'
            else:
                # if toggle was off, toggle it on
                #self.startTimer=time.clock()
                self.startControl = True
                self.state = 'GoForwardIfBlue'



    # overriding superclass's EditVideo method
    # can change self.cv_image here, and changes will be reflected on the video
    def EditVideo(self):
        
        if self.startControl:
            if(self.state == 'AdjustHeight'):
                self.adjustHeight()

            elif(self.state == 'HoverOnOrange'):
                self.HoverOnOrange()
            
            elif(self.state == 'FollowBlue'):
                self.FollowBlue()
            elif(self.state == 'GoForwardIfBlue'):
                self.GoForwardIfBlue()

    def HoverOnOrange(self):
        
        # calculating cx,cy,xspeed,yspeed
        orange_image=self.process.DetectColor(self.cv_image,'orange')
        self.cv_image=orange_image
        self.cx,self.cy=self.process.CenterofMass(orange_image)
        xspeed,yspeed=self.process.ApproximateSpeed(orange_image,self.cx,self.cy)
        

        # move drone corresponding to xspeed and yspeed at a fixed interval
        self.MoveFixedTime(xspeed,yspeed,0.1,0.03)
        #self.MoveTimer(xspeed,yspeed,0.2)     
            
    def FollowBlue(self):
        
        blue_image=self.process.DetectColor(self.cv_image,'blue')
        self.cv_image=blue_image
        x0,y0,angle=self.process.ShowLine(blue_image)

   #this function will go a certain speed for a set amount of time
    def MoveFixedTime(self,xspeed,yspeed,move_time,wait_time):
        
        xSetSpeed = None
        ySetSpeed = None

        if time.clock() > (self.startTimer+move_time+wait_time):
            xSetSpeed = xspeed
            ySetSpeed = yspeed
            self.startTimer=time.clock()

        elif time.clock() > (self.startTimer+move_time):
            xSetSpeed = 0
            ySetSpeed = 0

        if xSetSpeed != None and ySetSpeed != None:
            self.controller.SetCommand(xSetSpeed, ySetSpeed)

            # log info
            timeElapsed = (time.clock()-self.startTime)*1000
            currentDroneInfo = ("time: " +str(timeElapsed)+ " cx: " + str(self.cx) +
            " cy: " + str(self.cy) + " xspeed: " + str(xSetSpeed) 
            + " yspeed: " + str(ySetSpeed) + "\n")
            self.logFile.write(currentDroneInfo)


    #check if we are at the correct height and adjust
    def MoveFixedHeight(self,currentZ,stableTime):
        CLIMBSPEED = 0.7
        currentTime = time.clock()

        if(currentZ > 1.4):
            zVelocity = -CLIMBSPEED
            self.startTimer = time.clock()
        elif (currentZ < 1.3):
            zVelocity = CLIMBSPEED
            self.startTimer = time.clock()
        else:
            zVelocity = 0
            if(currentTime > (self.startTimer + stableTime)):
                self.state = 'hoverOnOrange'
                self.startTimer=time.clock()
        self.controller.SetCommand(z_velocity = zVelocity)

    # if 0.2 % of what the drone sees is orange, then it will go forward
    # orange corresponds to a hueMin of 0 and a hueMax of 50
    def GoForwardIfBlue(self):
        self.cv_image=self.process.DetectColor(self.cv_image,'blue')
        self.process.ShowLine(self.cv_image)

        orangeVisible = self.process.isHueDominant(self.cv_image, 100, 115, 0.1); 
        if orangeVisible:
            rospy.logwarn("go forward")
            self.controller.SetCommand(pitch = 0.1)
        else:
            rospy.logwarn("stop")
            self.controller.SetCommand(pitch = 0)

if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.spin()

    

