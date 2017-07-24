#!/usr/bin/env python

import rospy
import time
import datetime
import os

from os.path import expanduser

from drone_video import DroneVideo
from drone_controller import BasicDroneController
from processing_functions.process_video import ProcessVideo
from processing_functions.process_position import DronePosition
from processing_functions.logger import Logger
from processing_functions.picture_manager import PictureManager

import cv2
import numpy

# list of possible states to control drone
IDLE_STATE = "idle"
ADJUST_HEIGHT_STATE = "adjust_height"
HOVER_ORANGE_STATE = "hover_orange"
FOLLOW_BLUE_STATE = "follow_blue"
GO_FORWARD_IF_BLUE_STATE = "go_forward_if_blue"
FIX_LINE_ORIENTATION = "fix_line_orientation"


class TraceCircleController(DroneVideo):


    def __init__(self):

        # Call DroneVideo's constructor so we are subscribed to video feed 
        # will be converted to CVImage and edited by EditVideo() before shown.
        super(TraceCircleController,self).__init__()
        
        # Set up a timestamped folder inside Flight_Info that will have the pictures & log of this flight
        self.droneRecordPath= (expanduser("~")+"/drone_workspace/src/ardrone_lab/src/Flight_Info/"
        + datetime.datetime.now().strftime("%I:%M:%S%p_%a-%m-%d-%Y")+"_Flight"+"/")
        if not os.path.exists(self.droneRecordPath):
            os.makedirs(self.droneRecordPath)
        self.logger = Logger(self.droneRecordPath, "AR Drone Flight")
        self.logger.Start()

        # describes the algorithm currently being used to control the drone
        self.state = IDLE_STATE

        # initalizing helper objects
        self.process = ProcessVideo()
        self.controller = BasicDroneController("TraceCircle")
        self.pictureManager = PictureManager(self.droneRecordPath)
        self.startTimer = time.clock()
        

    # define keys to listen to here
    def KeyListener(self):
        
        key=cv2.waitKey(1) & 0xFF

        # toggles on/off which algorithm you want to execute with keypresses
        if key == ord('i'):
            
            self.IdleStateSwitch(HOVER_ORANGE_STATE)
        
        elif key == ord('b'):
            
            self.IdleStateSwitch(FOLLOW_BLUE_STATE)

        elif key == ord('m'):
            
            self.IdleStateSwitch(GO_FORWARD_IF_BLUE_STATE)

        elif key == ord('f'):

            self.IdleStateSwitch(FIX_LINE_ORIENTATION)

        elif key == ord('c'):

            self.captureFrame()
            

    # provides a "switch" between the drone being idle and being controlled
    def IdleStateSwitch(self, state):
        if self.state == IDLE_STATE:
            # if drone is idle, set the current state as specified in the parameter
            self.state = state
        else:
            # else, if drone is being controlled by a state, return back to idle
            self.controller.SetCommand(0,0,0,0)
            self.state = IDLE_STATE
        rospy.logwarn("Changed state to: " + self.state)
            

    # this is called every time a frame (in self.cv_image) is updated
    def EditVideo(self):
        
        if self.state == IDLE_STATE :
            # do nothing 
            pass
        elif self.state == ADJUST_HEIGHT_STATE:
            self.adjustHeight()

        elif self.state == HOVER_ORANGE_STATE:
            self.HoverOnOrange()
        
        elif self.state == FOLLOW_BLUE_STATE:
            self.FollowBlue()

        elif self.state == GO_FORWARD_IF_BLUE_STATE:
            self.GoForwardIfBlue()
        
        elif self.state == FIX_LINE_ORIENTATION:
            self.FixtoBlue()
        else:
            # if the current self.state is not one of the above, then it is undefined
            raise ValueError("State is undefined for Trace Circle Controller")


    # saves the current frame
    def captureFrame(self):
        pictureName = self.pictureManager.Capture(self.cv_image)
        rospy.logwarn("Saved picture as " + pictureName)


    # given that something orange is visible below the drone, will command the drone to hover directly over it 
    def HoverOnOrange(self):
        
        # calculating cx,cy,xspeed,yspeed
        orange_image=self.process.DetectColor(self.cv_image,'orange')
        self.cv_image=orange_image
        self.cx,self.cy=self.process.CenterofMass(orange_image)
        xspeed,yspeed=self.process.ApproximateSpeed(orange_image,self.cx,self.cy)
        

        # move drone corresponding to xspeed and yspeed at a fixed interval
        self.MoveFixedTime(xspeed,yspeed,move_time=0.1,wait_time=0.05)
           #0.1, 0.04 

    def FollowBlue(self):
        
        blue_image=self.process.DetectColor(self.cv_image,'blue')
        self.cv_image=blue_image
        angle=self.process.ShowLine(blue_image)
        cx,cy=self.process.CenterofMass(blue_image)


    #houghline transform on right half of image to fix orientation to blue after completing HoverOverOrange and taking image
    def FixtoBlue(self):
        blue_image=self.process.DetectColor(self.cv_image,'blue')
        self.cv_image=blue_image
        angle=self.process.ShowLine(blue_image[:,(blue_image.shape[1]/2):])
        cx,cy=self.process.CenterofMass(blue_image)
        yspeed,yawspeed=self.process.LineOrientation(blue_image,cy,angle)


    #fix the drone's orientation to face object before taking image
    def FaceObject(self):

        blue_image=self.process.DetectColor(self.cv_image,'blue')
        self.cv_image=blue_image
        angle=self.process.ShowLine(blue_image)
        cx,cy=self.process.CenterofMass(blue_image)
        xspeed,yawspeed=self.process.ObjectOrientation(blue_image,cx,angle)


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
            self.logger.Log("cx: " + str(self.cx) + " cy: " + str(self.cy) +
            " xspeed: " + str(xSetSpeed) + " yspeed: " + str(ySetSpeed))



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


    # if 0.2 % of what the drone sees is blue, then it will go forward
    # orange corresponds to a hueMin of 0 and a hueMax of 50
    def GoForwardIfBlue(self):

        self.cv_image=self.process.DetectColor(self.cv_image,'blue')
        self.process.ShowLine(self.cv_image)

        orangeVisible = self.process.isHueDominant(self.cv_image, 100, 115, 0.015); 

        if orangeVisible:
            rospy.logwarn("go forward")
            self.controller.SetCommand(pitch= 0.1)
        else:
            rospy.logwarn("stop")
            self.controller.SetCommand(pitch = 0)
            

    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        self.logger.Stop()


if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.on_shutdown(trace.ShutdownTasks)
    rospy.spin()

    

