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
HOVER_ORANGE_STATE = "hover_orange"
FACE_OBJECT = 'face_object'
FIX_TO_BLUE_LINE = "fix_to_blue_line"
FOLLOW_BLUE_STATE = "follow_blue"
ADJUST_HEIGHT_STATE = "adjust_height"

# the altitude & threashold at which all the algorithms that adjust height will
# go to, in mm
DRONE_ALTITUDE = 1000
ALT_THRESH = 75


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
        if key == ord('1'):
            
            self.IdleStateSwitch(HOVER_ORANGE_STATE)
            
        elif key == ord('2'):

            self.IdleStateSwitch(FACE_OBJECT)

        elif key == ord('3'):

            self.CaptureFrame()

        elif key == ord('4'):

            self.IdleStateSwitch(FIX_TO_BLUE_LINE)

        elif key == ord('5'):
            
            self.IdleStateSwitch(FOLLOW_BLUE_STATE)

        elif key == ord('h'):
            
            self.IdleStateSwitch(ADJUST_HEIGHT_STATE)
            

    # provides a "switch" between the drone being idle and being controlled
    def IdleStateSwitch(self, state):

        originalState = self.state
        if originalState == IDLE_STATE:
            # if drone is idle, set the current state as specified in the parameter
            self.state = state
        elif originalState == state:
            # if the current state is toggled again with the same state, return back to idle
            self.controller.SetCommand(0,0,0,0)
            self.state = IDLE_STATE
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

        elif self.state == FACE_OBJECT:

            self.FaceObject()

        elif self.state == FIX_TO_BLUE_LINE:

            self.FixToBlue()

        elif self.state == FOLLOW_BLUE_STATE:

            self.FollowBlue()
        
        elif self.state == ADJUST_HEIGHT_STATE:

            # Hovers to 950mm (0.95 m), with a tolerance of 100 mm (.1m)
            self.AdjustHeight(950, 100)

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

        # if there is orange in the screen, and the drone is in the middle, return true
        if orangeFound and xspeed == 0 and yspeed == 0 and zspeed == 0:
            rospy.logwarn("Done Hovering on Orange")
            return True
        else:
            rospy.logwarn("Trying to Hover on Orange")
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

        self.MoveFixedTime(xspeed, yspeed, yawspeed, zspeed, move_time=0.2, wait_time=0.04)

        if xspeed == 0 and yawspeed == 0 and found:
        #if xspeed == 0 and yawspeed == 0 and yspeed == 0 and found:
            rospy.logwarn("Facing Object")
            return True
        else:
            rospy.logwarn("Trying to Face Object")
            return False


    # Saves the current cv_image frame as a .png into the Flight_Info folder
    # Returns True once the picture has been taken
    def CaptureFrame(self):
        pictureName = self.pictureManager.Capture(self.cv_image)
        rospy.logwarn("Saved picture as " + pictureName)
        return True


    # Rotates the drone such that its front is perpendicular to the blue line below it;
    # makes blue line horizontal in bottom cam under the drone.
    # Returns False when it is still rotating to fix to the blue line
    # Returns True when it is fixed perpendicular to the blue line
    def FixToBlue(self):

        blue_image = self.process.DetectColor(self.cv_image,'blue')
        orange_image = self.process.DetectColor(self.cv_image,'orange')
        self.cv_image = blue_image

        # houghline transform on right half of image to fix orientation to blue after taking image
        angle = self.process.ShowLine(blue_image[:,3.5*(blue_image.shape[1]/10):],40)
        cx, cy, orangeFound = self.process.CenterofMass(orange_image)

        xspeed, yspeed, zspeed = self.process.ApproximateSpeed(blue_image, cx, cy, orangeFound,
        (self.flightInfo["altitude"])[1], DRONE_ALTITUDE, ALT_THRESH)
        yawspeed = self.process.LineOrientation(blue_image,cx,cy,angle)
        
        self.MoveFixedTime(xspeed*.3, yspeed*.3, yawspeed, 0, move_time=0.2, wait_time=0.04)

        # if there is blue in the screen, and the drone's front is perpendicular to the blue, return True
        if orangeFound and xspeed == 0 and yspeed == 0 and yawspeed == 0:
            rospy.logwarn("Done Fixing to Blue line")
            return True
        else:
            #rospy.logwarn("Trying to Fix to Blue Line")
            rospy.logwarn("xspeed : " + str(xspeed) + " yspeed: " + str(yspeed) +
            " yawspeed: " + str(yawspeed) + " zspeed: " + str(zspeed) )
            return False


    # Given that the drone's front is perpendicular to a blue line below it
    # the drone will fly right until its bottom camera cannot see blue anymore.
    # Returns False when it is still following the line
    # Returns True when it has finished following the blue line
    def FollowBlue(self):
        # we only consider the far right of the image to look for blue
        self.cv_image = self.process.CropVisible(self.cv_image, 
        int(len(self.cv_image[0])*(0/10)), 0, len(self.cv_image[0])/2, len(self.cv_image))

        
        self.cv_image=self.process.DetectColor(self.cv_image,'blue')

        angle=self.process.ShowLine(self.cv_image)
        
        # Blue will be deemed visible if .015% of the total screen is blue
        blueVisible = self.process.IsHueDominant(self.cv_image, 0, 360, 0.015); 

        if blueVisible:
            rospy.logwarn("Trying to Follow Blue")
            self.MoveFixedTime(-0.3, 0, 0, 0, move_time=0.1, wait_time=0.009)
            return False
        else:
            rospy.logwarn("Done Following Blue")
            self.MoveFixedTime(0 , 0 , 0, 0, move_time=0.1, wait_time=0.009)
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

        self.MoveFixedTime(0 , 0 , 0, zVelocity, move_time=0.1, wait_time=0.009)


    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        self.logger.Stop()


if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.on_shutdown(trace.ShutdownTasks)
    rospy.spin()

    

