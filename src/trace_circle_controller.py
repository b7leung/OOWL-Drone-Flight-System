#!/usr/bin/env python

# importing python modules
import rospy
import time
import datetime
import os
import cv2
from os.path import expanduser

# importing drone specific modules
from drone_video import DroneVideo
from drone_controller import BasicDroneController
from flightstats_receiver import FlightstatsReceiver
from processing_functions.process_video import ProcessVideo
from processing_functions.process_position import DronePosition
from processing_functions.logger import Logger
from processing_functions.picture_manager import PictureManager


# list of possible states to control drone
IDLE_STATE = "idle"
ADJUST_HEIGHT_STATE = "adjust_height"
HOVER_ORANGE_STATE = "hover_orange"
FOLLOW_BLUE_STATE = "follow_blue"
GO_FORWARD_IF_BLUE_STATE = "go_forward_if_blue"
FIX_LINE_ORIENTATION = "fix_line_orientation"


# TraceCircleController "is-a" Drone Video display and a flightstats receiver. It uses
# this info to ultimately control the drone
class TraceCircleController(DroneVideo, FlightstatsReceiver):


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

        elif key == ord('h'):
            
            self.IdleStateSwitch(ADJUST_HEIGHT_STATE)
        elif key == ord('f'):

            self.IdleStateSwitch(FIX_LINE_ORIENTATION)

        elif key == ord('c'):

            self.captureFrame()
            

    # provides a "switch" between the drone being idle and being controlled
    def IdleStateSwitch(self, state):
        originalState = self.state
        if self.state == IDLE_STATE:
            # if drone is idle, set the current state as specified in the parameter
            self.state = state
        else:
            # else, if drone is being controlled by a state, return back to idle
            self.controller.SetCommand(0,0,0,0)
            self.state = IDLE_STATE
        rospy.logwarn("======= Changed from " + originalState + " state to: " +
        self.state + " state =======")
            

    # this is called every time a frame (in self.cv_image) is updated
    def EditVideo(self):
        
        if self.state == IDLE_STATE :
            # do nothing 
            pass
        elif self.state == ADJUST_HEIGHT_STATE:
            # Hovers to 950mm (0.95 m), with a tolerance of 100 mm (.1m)
            self.AdjustHeight(950, 100)

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
        self.MoveFixedTime(xspeed, yspeed, 0, 0, move_time=0.1, wait_time=0.04)
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
    def MoveFixedTime(self, xSpeed, ySpeed, yawSpeed, zSpeed, move_time, wait_time):
        
        xSetSpeed = None
        ySetSpeed = None
        yawSetSpeed = None
        zSetSpeed = None

        if time.clock() > (self.startTimer+move_time+wait_time):
            xSetSpeed = xSpeed
            ySetSpeed = ySpeed
            yawSetSpeed = yawSpeed
            zSetSpeed = zSpeed
            self.startTimer=time.clock()

        elif time.clock() > (self.startTimer+move_time):
            xSetSpeed = 0
            ySetSpeed = 0
            yawSetSpeed = 0
            zSetSpeed = 0

        if (xSetSpeed != None and ySetSpeed != None and
        yawSetSpeed != None and zSetSpeed != None):
            self.controller.SetCommand(xSetSpeed, ySetSpeed, yawSetSpeed, zSetSpeed)

            # log info
            #self.logger.Log("cx: " + str(self.cx) + " cy: " + str(self.cy) +
            self.logger.Log(
            " xSpeed: " + str(xSetSpeed) + " ySpeed: " + str(ySetSpeed)
            + " yawSpeed: " +str(yawSetSpeed) + " zSpeed: " + str(zSetSpeed) )



    # adjusts the drone to the desired altitude within a tolerance, in mm
    def AdjustHeight(self, desiredAltitude, tolerance):

        climbSpeed = 0.25

        if (self.flightInfo["altitude"])[1] < (desiredAltitude - tolerance):
            zVelocity = climbSpeed
            rospy.logwarn("go up")
        elif (self.flightInfo["altitude"])[1] > (desiredAltitude + tolerance):
            zVelocity = climbSpeed * -1
            rospy.logwarn("go down")
        else:
            zVelocity = 0
            rospy.logwarn("stay in place")

        #self.controller.SetCommand(z_velocity = zVelocity)
        self.MoveFixedTime(0 , 0 , 0, zVelocity, move_time=0.1, wait_time=0.009)


    # if 0.2 % of what the drone sees is blue, then it will go forward
    # orange corresponds to a hueMin of 0 and a hueMax of 50
    def GoForwardIfBlue(self):

        self.cv_image=self.process.DetectColor(self.cv_image,'blue')
        self.process.ShowLine(self.cv_image)

        orangeVisible = self.process.isHueDominant(self.cv_image, 100, 115, 0.015); 

        if orangeVisible:
            rospy.logwarn("go forward")
            self.MoveFixedTime(0 , 0.1 , 0, 0, move_time=0.1, wait_time=0.009)
            #self.controller.SetCommand(pitch= 0.1)
        else:
            rospy.logwarn("stop")
            self.MoveFixedTime(0 , 0 , 0, 0, move_time=0.1, wait_time=0.009)
            #self.controller.SetCommand(pitch= 0.0)
            

    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        self.logger.Stop()


if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.on_shutdown(trace.ShutdownTasks)
    rospy.spin()

    

