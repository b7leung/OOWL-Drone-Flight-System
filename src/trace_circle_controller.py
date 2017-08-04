#!/usr/bin/env python

# importing python modules
import rospy
import time
import datetime
import os
import cv2
import numpy
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
HOVER_ORANGE_STATE = "hover_orange"
FACE_OBJECT_STATE = 'face_object'
PREPARE_CAPTURE_STATE = 'prepare_capture'
CAPTURE_FRAME_STATE = 'capture_frame'
FIX_TO_BLUE_STATE = "fix_to_blue"
FOLLOW_BLUE_STATE = "follow_blue"
AUTO_CIRCLE_STATE = "auto_circle"

# the altitude & threashold at which all the algorithms that adjust height will
# go to, in mm
DRONE_ALTITUDE = 1300
ALT_THRESH = 100


# TraceCircleController "is-a" Drone Video display and a flightstats receiver. It uses
# this info to ultimately control the drone
class TraceCircleController(DroneVideo, FlightstatsReceiver):


    def __init__(self):

        # Call DroneVideo's constructor so we are subscribed to video feed 
        # will be converted to CVImage and edited by EditVideo() before shown.
        super(TraceCircleController,self).__init__()
        
        # Set up a timestamped folder inside Flight_Info that will have the pictures & log of this flight
        self.droneRecordPath= (expanduser("~")+"/drone_workspace/src/ardrone_lab/src/Flight_Info/"
        + datetime.datetime.now().strftime("%m-%d-%Y__%H:%M:%S, %A")+"_Flight"+"/")
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
        self.autonomyState = False
        self.stateCounter = 0


    # define keys to listen to here. When key is presses, does any necessary preparation, then goes to state
    def KeyListener(self):
        
        key=cv2.waitKey(1) & 0xFF

        # toggles on/off which algorithm you want to execute with keypresses
        if key == ord('1'):
            
            self.IdleStateSwitch(HOVER_ORANGE_STATE)
            
        elif key == ord('2'):

            self.IdleStateSwitch(FACE_OBJECT_STATE)

        elif key == ord('3'):

            self.IdleStateSwitch(CAPTURE_FRAME_STATE)

        elif key == ord('4'):

            self.IdleStateSwitch(FIX_TO_BLUE_STATE)

        elif key == ord('5'):
            
            self.IdleStateSwitch(FOLLOW_BLUE_STATE)

        elif key == ord('c'):
            
            # list of algorithms to execute, in order. They are in format:
            # (algorithm method name, frames to wait to make sure algorithm is finished).
            self.algorithmOrdering = [(self.HoverOnOrange, 20), (self.FaceObject, 20), (self.FixToBlue, 10),
            (self.PrepareCapture,1), (self.CaptureFrame,1), (self.FollowBlue, 5)]
            self.currAlgorithm = algorithmOrdering[0]
            self.currAlgorithmFinished= False
            self.currAlgorithmCounter = 0
            self.IdleStateSwitch(AUTO_CIRCLE_STATE)

        elif key == ord('s'):

            self.IdleStateSwitch(HOVER_ORANGE_STATE)
            self.autonomyState = True


    # provides a "switch" between the drone being idle and being controlled
    def IdleStateSwitch(self, state):

        originalState = self.state
        if originalState == IDLE_STATE:
            # if drone is idle, set the current state as specified in the parameter
            self.state = state
        elif originalState == state or self.autonomyState:
            # if the current state is toggled again with the same state, return back to idle
            self.controller.SetCommand(0,0,0,0)
            self.state = IDLE_STATE
            self.autonomyState = False
        else:
            # if the drone was in an non-idle state and the new state is different than the original,
            # switch to it
            self.controller.SetCommand(0,0,0,0)
            self.state = state
            
        rospy.logwarn('======= Changed from "' + originalState + '" state to "' +
        self.state + '" state =======')
            

    # this is called every time a frame (in self.cv_image) is updated
    def EditVideo(self):
        
        if self.state == IDLE_STATE :

            # do nothing 
            pass

        elif self.state == HOVER_ORANGE_STATE:

            self.HoverOnOrange()

        elif self.state == FACE_OBJECT_STATE:

            self.FaceObject()

        elif self.state == FIX_TO_BLUE_STATE:

            self.FixToBlue()

        elif self.state == PREPARE_CAPTURE_STATE:
            
            self.PrepareCapture()

        elif self.state == CAPTURE_FRAME_STATE:

            self.CaptureFrame()

        elif self.state == FOLLOW_BLUE_STATE:

            self.FollowBlue()
               
        elif self.state ==  AUTO_CIRCLE_STATE:
            

            """
            self.currAlgorithmFinished = self.currState()
            if self.currStateFinished:
                self.currStateCounter = self.currStateCounter + 1
            if self."""
            
            
            pass

        else:

            # if the current self.state is not one of the above, then it is undefined
            raise ValueError("State is undefined for Trace Circle Controller")


    # Given that something orange is visible below the drone, will command 
    # the drone to hover directly over it 
    # Returns False if algorithm is still running and drone isn't on orange yet
    # Returns True if algorithm is finished and drone is now on orange
    def HoverOnOrange(self):
        
        # converting to a segmented orange image and calculating the corresponding cx, cy, xspeed, yspeed
        self.cv_image = self.process.DetectColor(self.cv_image, 'orange')
        cx, cy, orangeFound = self.process.CenterofMass(self.cv_image)
        xspeed, yspeed, zspeed = self.process.ApproximateSpeed(self.cv_image, cx, cy, 
        orangeFound,(self.flightInfo["altitude"])[1], DRONE_ALTITUDE, ALT_THRESH)


        # move drone corresponding to xspeed and yspeed at a fixed interval
        self.MoveFixedTime(xspeed, yspeed, 0, zspeed, 0.1, 0.04)

        # logging cx + cy
        self.logger.Log(str(cx) + " " + str(cy))

        # if there is orange in the screen, and the drone is in the middle, return true
        if orangeFound and xspeed == 0 and yspeed == 0 and zspeed == 0:

            rospy.logwarn("Done Hovering on Orange")
            self.stateCounter += 1
            if(self.stateCounter >= 15 and self.autonomyState):
                self.stateCounter = 0
                self.state = FACE_OBJECT_STATE
            return True

        else:

            rospy.logwarn("Trying to Hover on Orange")
            self.stateCounter = 0
            return False


    # Fix the drone's orientation to face object before taking image
    # Makes green line vertical in bottom cam under the drone
    # Returns False if drone isn't facing object
    # Returns True if drone is facing object; parallel to the green line
    def FaceObject(self):

        green_image = self.process.DetectColor(self.cv_image, 'green')
        orange_image = self.process.DetectColor(self.cv_image,'orange')
        self.cv_image = green_image

        # trying to be parallel to the green line, while being over orange checkpoint
        angle = self.process.ShowLine(green_image, thresh=40)
        cx, cy, found=self.process.CenterofMass(orange_image)

        xspeed,yspeed,zspeed = self.process.ApproximateSpeed(green_image,cx,cy,found,
        (self.flightInfo["altitude"])[1], DRONE_ALTITUDE, ALT_THRESH)
        yawspeed = self.process.ObjectOrientation(green_image, cx, cy, angle)

        self.MoveFixedTime(xspeed, yspeed, yawspeed, 0, move_time=0.2, wait_time=0.04)

        if xspeed == 0 and yspeed == 0 and yawspeed == 0 and found:

            rospy.logwarn("Facing Object")
            self.stateCounter += 1
            if(self.stateCounter >= 10 and self.autonomyState):
                self.stateCounter = 0
                self.state = PREPARE_CAPTURE_STATE
            return True

        else:

            rospy.logwarn("Trying to Face Object")
            self.stateCounter = 0
            return False

    
    # Prepares the drone for capturing a photo; Toggles the camera,
    # and tells the program to wait while the drone is switching its cameras
    def PrepareCapture(self):
        
        self.controller.ToggleCamera()
        self.state = CAPTURE_FRAME_STATE
        # causes algorithm to wait 20 frames before executing the next state
        self.Sleep(20)


    # Switches to front camera, saves the current cv_image frame 
    # as a .png into the Flight_Info folder, then switches to bottom again
    # Returns True once the picture has been taken
    def CaptureFrame(self):
        pictureName = self.pictureManager.Capture(self.cv_image)
        rospy.logwarn("Saved picture as " + pictureName)
        self.controller.SwitchCamera(1)
        #if self.autonomyState:
        self.state = FIX_TO_BLUE_STATE
        return True


    # Rotates the drone such that its front is perpendicular to the blue line below it;
    # makes blue line horizontal in bottom cam under the drone.
    # Returns False when it is still rotating to fix to the blue line
    # Returns True when it is fixed perpendicular to the blue line
    def FixToBlue(self):

        blue_image = self.process.DetectColor(self.cv_image,'blue')
        orange_image = self.process.DetectColor(self.cv_image,'orange')
        # houghline transform on right half of image to fix orientation to blue after taking image
        angle = self.process.ShowLine(blue_image,20,110,thresh=45)
        #angle = self.process.ShowLine(blue_image[:,10*(blue_image.shape[1]/10):],thresh=40)
        cx, cy, orangeFound = self.process.CenterofMass(orange_image)

        xspeed, yspeed, zspeed = self.process.ApproximateSpeed(blue_image, cx, cy, orangeFound,
        (self.flightInfo["altitude"])[1], DRONE_ALTITUDE, ALT_THRESH)
        yawspeed = self.process.LineOrientation(blue_image,cx,cy,angle)
        
        self.MoveFixedTime(xspeed*.7, yspeed*.7, yawspeed, 0, move_time=0.2, wait_time=0.04)

        #blue_image = cv2.cvtColor(blue_image, cv2.COLOR_BGR2GRAY)
        #rospy.logwarn("blue image: " + str(numpy.shape(blue_image))+ " edges: " + str(numpy.shape(edges)))
        #self.cv_image = numpy.hstack((blue_image,edges))
        self.cv_image = blue_image

        # if there is blue in the screen, and the drone's front is perpendicular to the blue, return True
        if orangeFound and xspeed == 0 and yspeed == 0 and yawspeed == 0:

            rospy.logwarn("Done Fixing to Blue line")
            self.stateCounter += 1
            if(self.stateCounter >= 20 and self.autonomyState):
                self.stateCounter = 0
                self.state = FOLLOW_BLUE_STATE
            return True

        else:

            rospy.logwarn("Trying to Fix to Blue Line")
            self.stateCounter = 0
            return False


    # Given that the drone's front is perpendicular to a blue line below it
    # the drone will fly right until its bottom camera cannot see blue anymore.
    # Returns False when it is still following the line
    # Returns True when it has finished following the blue line
    def FollowBlue(self):
        # we only consider the far right of the image to look for blue
        self.cv_image = self.process.CropVisible(self.cv_image, 
        int(len(self.cv_image[0])*(0/10)), 0, len(self.cv_image[0]), len(self.cv_image))

        
        self.cv_image=self.process.DetectColor(self.cv_image,'blue')

        angle=self.process.ShowLine(self.cv_image,75,120)
        
        # Blue will be deemed visible if .015% of the total screen is blue
        #blueVisible = self.process.IsHueDominant(self.cv_image, 0, 360, 0.015); 


        if angle != None:

            rospy.logwarn("Trying to Follow Blue")
            self.MoveFixedTime(-0.3, 0, 0, 0, move_time=0.1, wait_time=0.009)
            self.stateCounter = 0
            return False

        else:

            rospy.logwarn("Done Following Blue")
            self.MoveFixedTime(0 , 0 , 0, 0, move_time=0.1, wait_time=0.009)
            self.stateCounter += 1
            if(self.stateCounter >= 20 and self.autonomyState):
                self.stateCounter = 0
                self.state = HOVER_ORANGE_STATE
            return True
            

    # this function will go a certain speed for a set amount of time
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
            """
            self.logger.Log(
            " xSpeed: " + str(xSetSpeed) + " ySpeed: " + str(ySetSpeed)
            + " yawSpeed: " +str(yawSetSpeed) + " zSpeed: " + str(zSetSpeed) )
            """


    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        self.logger.Stop()


if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.on_shutdown(trace.ShutdownTasks)
    rospy.spin()

    

