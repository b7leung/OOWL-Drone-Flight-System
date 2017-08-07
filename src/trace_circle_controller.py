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
from processing_functions.pid_controller import PIDController

from state_machine import StateMachine
from drone_directives.AbstractDroneDirective import * 
from drone_directives.HoverColorDirective import * 
from drone_directives.OrientVLineDirective import *
from drone_directives.OrientPLineDirective import *
from drone_directives.FollowLineDirective import *

# list of possible state machines that can be used to control drone
IDLE_MACHINE = "idle"
HOVER_ORANGE_MACHINE = "hover_orange"
FACE_OBJECT_MACHINE = 'face_object'
FIX_TO_BLUE_LINE_MACHINE = "fix_to_blue"
FOLLOW_BLUE_LINE_MACHINE = "follow_blue"
AUTO_CIRCLE_MACHINE= "auto_circle"
PID_MACHINE = 'pid_state'
RETURN_MACHINE = "return"


# A class that has access to the drone video feed and navdata. It uses this information to feed into
# any one of its defined state machines, which will tell this class how to control the drone.
# The particular state machine to use can be changed at run time.
class TraceCircleController(DroneVideo, FlightstatsReceiver):


    def __init__(self):

        # getting access to elements in DroneVideo and FlightstatsReciever
        super(TraceCircleController,self).__init__()
        
        # Seting up a timestamped folder inside Flight_Info that will have the pictures & log of this flight
        self.droneRecordPath= (expanduser("~")+"/drone_workspace/src/ardrone_lab/src/Flight_Info/"
        + datetime.datetime.now().strftime("%m-%d-%Y__%H:%M:%S, %A")+"_Flight"+"/")
        if not os.path.exists(self.droneRecordPath):
            os.makedirs(self.droneRecordPath)
        self.logger = Logger(self.droneRecordPath, "AR Drone Flight")
        self.logger.Start()

        # initalizing the state machine that will handle which algorithms to run at which time;
        # the results of the algorithms will be used to control the drone
        self.stateMachine = StateMachine()
        
        # drone starts without any machine loaded, so that it can be controlled using the keyboard
        self.currMachine = None

        # initalizing helper objects
        self.process = ProcessVideo()
        self.controller = BasicDroneController("TraceCircle")
        self.pid = PIDController()
        self.pictureManager = PictureManager(self.droneRecordPath)
        self.startTimer = time.clock()

        self.lastLocation = (None,None)
        self.temp = False
        

    # Each state machine that trace circle controller can use is defined here;
    # When key is pressed, define the machine to be used and switch over to it.
    # Machines are defined as array of tuples. Each tuple represents a state's directive and duration
    # in the format (stateobject, stateduration). stateobject must subclass AbstractDroneDirective
    def KeyListener(self):
        
        key=cv2.waitKey(1) & 0xFF

        if key == ord('1'):
            
            self.MachineSwitch( [(HoverColorDirective('orange', 700), 0)], HOVER_ORANGE_MACHINE )
            
        elif key == ord('2'):

            self.MachineSwitch( [(OrientVLineDirective('green', 'orange', 700), 0)], FACE_OBJECT_MACHINE)

        elif key == ord('3'):

            #self.IdleMachineSwitch(CAPTURE_FRAME_STATE)
            pass

        elif key == ord('4'):

            self.MachineSwitch( [(OrientPLineDirective('blue', 'orange', 700), 0)], FIX_TO_BLUE_LINE_MACHINE)

        elif key == ord('5'):
            
            self.MachineSwitch( [(FollowLineDirective('blue'), 0)], FOLLOW_BLUE_LINE_MACHINE )

        elif key == ord('s'):
            
            machineDef = [
            ( HoverColorDirective('orange', 700), 15 ),
            ( OrientVLineDirective('green', 'orange', 700), 7 ),
            ( OrientPLineDirective('blue', 'orange', 700), 7 ),
            ( FollowLineDirective('blue'), 10)
            ]
            
            self.MachineSwitch( machineDef, AUTO_CIRCLE_MACHINE)
            
        elif key == ord('p'):
            
            pass


    # Taking in some machine's definition of states and a string name,
    # provides a "switch" for loading and removing the machines that
    # trace circle controller uses to control the drone
    def MachineSwitch(self, newMachineDefinition, newMachineName):

        originalMachine = self.currMachine

        if originalMachine == None:
            # if no machine is loaded (drone is idle), set 
            # the current machine as specified in the parameter
            self.stateMachine.SetStates(newMachineDefinition)
            self.currMachine = newMachineName

        elif originalMachine == newMachineName:
            # if the current machine is toggled again with the same machine,
            # remove the machine and return back to the idle 
            self.controller.SetCommand(0,0,0,0)
            self.currMachine = None

        else:
            # if the drone was in an non-idle state and the new state is different than the original,
            # directly switch to it
            self.controller.SetCommand(0,0,0,0)
            self.stateMachine.SetStates(newMachineDefinition)
            self.currMachine = newMachineName
            
        rospy.logwarn('======= Changed from the "' + str(originalMachine) + '" machine to the "' +
        str(self.currMachine) + '" machine =======')
            

    # This is called every time a frame (in self.cv_image) is updated.
    # Runs an iteration of the current state machine to get the next set of instructions, depending on the 
    # machine's current state.
    def ReceivedVideo(self):
        
        # If no machine is loaded, then trace circle controller does nothing 
        # (so that the drone may be controlled with the keyboard)
        if self.currMachine == None:
            pass
        else:
            # retrieving the instructions for whatever state the machine is in 
            # and commanding the drone to move accordingly
            droneInstructions, segImage = self.stateMachine.GetUpdate(self.cv_image, self.flightInfo)
            self.cv_image = segImage
            self.MoveFixedTime(droneInstructions[0], droneInstructions[1],
            droneInstructions[2], droneInstructions[3], 0.1, 0.04)


    def RunPIDController(self):

        orange_image = self.process.DetectColor(self.cv_image, 'orange')
        cx, cy = self.process.CenterOfMass(orange_image)
        self.cv_image = orange_image 
        self.pid.UpdateDeltaTime()
        self.pid.SetPoint(orange_image)
        self.pid.SetPIDConstants(0.4, 0.0, 0.0)
        self.pid.UpdateError(cx,cy)
        x_P, y_P, x_I, y_I, x_D, y_D = self.pid.SetPIDTerms()
        xspeed, yspeed = self.pid.GetPIDValues()

        #rospy.logwarn("X Terms: "+str(x_P)+", " + str(x_I)+", "+ str(x_D))
        #rospy.logwarn("Y Terms: "+str(y_P)+", " + str(y_I)+", "+ str(y_D))

        rospy.logwarn("xPID, yPID: "+str(xspeed) +", " + str(yspeed))
        self.controller.SetCommand(xspeed,yspeed, 0, 0)
        #self.MoveFixedTime(xspeed, yspeed, 0 ,0, 0.1, 0.01)

    
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
        if self.autonomyState:
            self.state = FIX_TO_LINE_STATE
        return True


    # This state will push the drone back towards the target location if 
    # it has drifted out of the cameras vision
    def ReturnState(self, platformColor = 'orange'):
        platform_image = self.process.DetectColor(self.cv_image, platformColor)
        self.cv_image = platform_image
        numrows,numcols,channels=self.cv_image.shape
        
        cx = self.lastLocation[0]
        cy = self.lastLocation[1]

        centerx=numcols/2
        centery=numrows/2
        width=120
        height=120
        xlower=centerx-width #left xvalue
        ylower=centery-height #"top" yvalue
        xupper=centerx+width #right xvalue
        yupper=centery+height #"bottom" yvalue
        hasPlatform = self.process.IsHueDominant(platform_image, 0, 360, 0.2)            
        
        if( not hasPlatform):
            self.stateCounter += 1
            rospy.logwarn("currently returning, statecounter: "+ str(self.stateCounter))
            if (self.stateCounter > 2) or (cx>xlower and cx < xupper) or (cy > ylower and cy < yupper):
                self.state = self.lastState
                rospy.logwarn(str(self.lastState))
                self.stateCounter = 0
                self.lastLocation = (None,None)
                self.MoveFixedTime(0, 0, 0 ,0, 0.1, 0.04)

                return False
            xspeed, yspeed, zspeed = self.process.ApproximateSpeed(self.cv_image, cx, cy, 
            (self.flightInfo["altitude"])[1], DRONE_ALTITUDE)
            self.MoveFixedTime(xspeed, yspeed, 0 ,0, 0.1, 0.04)
            return False
        else:
            rospy.logwarn("done returning")
            self.state = self.lastState
            self.stateCounter = 0
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

    

