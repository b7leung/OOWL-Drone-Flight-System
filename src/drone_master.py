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
from state_machine import StateMachine
from processing_functions import *
from drone_directives import *


# list of possible state machines that can be used to control drone
IDLE_MACHINE = "idle"
HOVER_ORANGE_MACHINE = "hover_orange"
FACE_OBJECT_MACHINE = 'face_object'
CAPTURE_PHOTO_MACHINE = 'capture_photo'
FIX_TO_BLUE_LINE_MACHINE = "fix_to_blue"
FOLLOW_BLUE_LINE_MACHINE = "follow_blue"
AUTO_CIRCLE_MACHINE= "auto_circle"
PID_HOVER_ORANGE_MACHINE = 'pid_hover_orange'
RETURN_MACHINE = "return"


# A class that has access to the drone video feed and navdata. It uses this information to feed into
# any one of its defined state machines, which will tell this class how to control the drone.
# The particular state machine to use can be changed at run time.
class DroneMaster(DroneVideo, FlightstatsReceiver):


    def __init__(self):

        # getting access to elements in DroneVideo and FlightstatsReciever
        super(DroneMaster,self).__init__()
        
        # Seting up a timestamped folder inside Flight_Info that will have the pictures & log of this flight
        self.droneRecordPath= (expanduser("~")+"/drone_workspace/src/ardrone_lab/src/Flight_Info/"
        + datetime.datetime.now().strftime("%m-%d-%Y__%H:%M:%S, %A")+"_Flight"+"/")
        if not os.path.exists(self.droneRecordPath):
            os.makedirs(self.droneRecordPath)
        self.logger = Logger(self.droneRecordPath, "AR Drone Flight")
        self.logger.Start()

        # initalizing the state machine that will handle which algorithms to run at which time;
        # the results of the algorithms will be used to control the drone
        self.stateMachine = StateMachine(None)
        #self.stateMachine = StateMachine((ReturnToColorDirective('orange'),30))
        
        # drone starts without any machine loaded, so that it can be controlled using the keyboard
        self.currMachine = None

        # initalizing helper objects
        self.process = ProcessVideo()
        self.controller = BasicDroneController("TraceCircle")
        self.startTimer = time.clock()


    # Each state machine that drone mastercan use is defined here;
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

            # toggles cameras back and forth to take a photo once every 200 frames
            # The 7 frame idles in between are to give the drone time to switch the camera
            machineDef = [
            (ToggleCameraDirective(), 1),
            (IdleDirective(), 7),
            (CapturePhotoDirective(self.droneRecordPath), 1),
            (ToggleCameraDirective(), 1),
            (IdleDirective(), 200)
            ]

            self.MachineSwitch( machineDef, CAPTURE_PHOTO_MACHINE)

        elif key == ord('4'):

            self.MachineSwitch( [(OrientPLineDirective('blue', 'orange', 700), 0)], FIX_TO_BLUE_LINE_MACHINE)

        elif key == ord('5'):
            
            self.MachineSwitch( [(FollowLineDirective('blue'), 0)], FOLLOW_BLUE_LINE_MACHINE )

        elif key == ord('s'):
            
            # does the entire circle algorithm, in order.
            machineDef = [
            ( HoverColorDirective('orange', 1100), 15 ),
            ( OrientVLineDirective('green', 'orange', 1000), 8 ),
            ( ToggleCameraDirective(), 1 ),
            # give drone time to switch cameras
            ( IdleDirective(), 10 ),
            ( CapturePhotoDirective(self.droneRecordPath), 1 ),
            ( ToggleCameraDirective(), 1 ),
            ( IdleDirective(), 10 ),
            ( OrientPLineDirective('blue', 'orange', 1000), 8 ),
            ( FollowLineDirective('blue'), 20 )
            ]
            
            self.MachineSwitch( machineDef, AUTO_CIRCLE_MACHINE)
            
        elif key == ord('p'):

            self.MachineSwitch( [(PIDHoverColorDirective('orange'), 0)], PID_HOVER_ORANGE_MACHINE )


    # Taking in some machine's definition of states and a string name,
    # provides a "switch" for loading and removing the machines that
    # drone master uses to control the drone
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
  
        # If no machine is loaded, then drone master does nothing 
        # (so that the drone may be controlled with the keyboard)
        if self.currMachine == None:
            pass
        else:
            # retrieving the instructions for whatever state the machine is in 
            # and commanding the drone to move accordingly
            droneInstructions, segImage = self.stateMachine.GetUpdate(self.cv_image, self.flightInfo)
            self.cv_image = segImage
            self.MoveFixedTime(droneInstructions[0], droneInstructions[1],
            droneInstructions[2], droneInstructions[3], 0.15, 0.04)


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
        #if there is no wait time then don't reset speed to 0
            if(wait_time != 0):
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


    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        self.logger.Stop()


if __name__=='__main__':
    
    rospy.init_node('DroneMaster')
    master = DroneMaster()
    rospy.on_shutdown(master.ShutdownTasks)
    rospy.spin()

    

