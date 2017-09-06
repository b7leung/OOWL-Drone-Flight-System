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
from processing_functions.picture_manager import PictureManager

# list of possible state machines that can be used to control drone
IDLE_MACHINE = "idle"
HOVER_ORANGE_MACHINE = "hover_orange"
FACE_OBJECT_MACHINE = 'face_object'
CAPTURE_PHOTO_MACHINE = 'capture_photo'
FIX_TO_BLUE_LINE_MACHINE = "fix_to_blue"
FOLLOW_BLUE_LINE_MACHINE = "follow_blue"
AUTO_CIRCLE_MACHINE= "auto_circle"
PID_AUTO_CIRCLE_MACHINE= "pid_auto_circle"
PID_HOVER_ORANGE_MACHINE = 'pid_hover_orange'
SELF_CORRECTING_TAKEOFF_MACHINE = "self_correcting_takeoff"
PID_OBJECT_ORIENT_MACHINE = 'pid_object_orient'
TEST_MACHINE = 'test_machine'


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

        #import PID and color constants
        self.settingsPath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/calibrater_settings.txt"

        # initalizing the state machine that will handle which algorithms to run at which time;
        # the results of the algorithms will be used to control the drone
        self.stateMachine = StateMachine()
        
        # drone starts without any machine loaded, so that it can be controlled using the keyboard
        self.currMachine = None

        # initalizing helper objects
        self.pictureManager = PictureManager(self.droneRecordPath)
        self.process = ProcessVideo()
        self.controller = BasicDroneController("TraceCircle")
        self.startTimer = time.clock()
        self.waitTime = 0
        self.moveTime = 0


    # Each state machine that drone mastercan use is defined here;
    # When key is pressed, define the machine to be used and switch over to it.
    # Machines are defined as array of tuples. Each tuple represents a state's directive and duration
    # in the format (stateobject, stateduration). stateobject must subclass AbstractDroneDirective
    def KeyListener(self):
        
        key=cv2.waitKey(1) & 0xFF

        if key == ord('t'):
            
            # self correcting takeoff
            
            self.moveTime = 0.0
            self.waitTime = 0.0

            init = [
            ( FlatTrimDirective(), 1), ( IdleDirective("pause for flat trim"), 10 ),
            ( ToggleCameraDirective(), 1 ), ( IdleDirective("pause to toggle camera"), 10 ),
            ( TakeoffDirective(), 1), ( IdleDirective("pause for takeoff"), 130 ),
            ( ReturnToOriginDirective(100), 15 )
            #( HoverColorDirective('orange', altitude), 10 )
            ]

            self.MachineSwitch( init, None, 0, None, None, SELF_CORRECTING_TAKEOFF_MACHINE)

        elif key == ord('1'):

            self.moveTime = 0.15
            self.waitTime = 0.08
            alg = [(HoverColorDirective('orange', 700), 0)]
            algCycles = -1
            self.MachineSwitch( None , alg, algCycles, None, None, HOVER_ORANGE_MACHINE )
            
        elif key == ord('2'):

            self.moveTime = 0.22
            self.waitTime = 0.12
            alg = [( OrientLineDirective( 'PARALLEL', 'green', 'orange', 700), 0 )]
            algCycles = -1
            self.MachineSwitch( None, alg, algCycles, None, None, FACE_OBJECT_MACHINE)

        elif key == ord('3'):

            pictureName = self.pictureManager.Capture(self.cv_image)
            rospy.logwarn("Saved picture as " + pictureName)

        elif key == ord('4'):

            self.moveTime = 0.15
            self.waitTime = 0.08
            alg = [ ( OrientLineDirective('PERPENDICULAR', 'blue', 'orange', 700), 0 ) ]
            algCycles = -1

            self.MachineSwitch( None, alg, algCycles, None, None, FIX_TO_BLUE_LINE_MACHINE)

        elif key == ord('5'):
            
            self.moveTime = 0.15
            self.waitTime = 0.08
            alg = [
            (FollowLineDirective('blue'), 0)
            ]
            algCycles = -1
            self.MachineSwitch( None, alg, algCycles, None, None, FOLLOW_BLUE_LINE_MACHINE)

        elif key == ord('s'):

            # does the entire circle algorithm, in order.

            self.moveTime = 0.20
            self.waitTime = 0.10
            altitude = 1000
            
            init = [
            ( SetupDirective(), 1), ( IdleDirective("Pause for setup"), 10 ),
            ( FlatTrimDirective(), 1), ( IdleDirective("Pause for flat trim"), 10 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective(" Pause for seting camera"), 10 ),
            ( TakeoffDirective(), 1), ( IdleDirective("Pause for takeoff"), 120 ),
            ( ReturnToOriginDirective('orange',50), 7 ),
            ( FindPlatformAltitudeDirective('orange', altitude + 200), 5),
            ]

            alg = [
            ( OrientLineDirective( 'PARALLEL', 'green', 'orange', altitude ), 10 ),
            ( SetCameraDirective("FRONT"), 1 ), ( IdleDirective("Pause for setting camera"), 18 ),
            ( CapturePhotoDirective(self.droneRecordPath), 1 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective("Pause for setting camera"), 15 ),
            ( OrientLineDirective('PERPENDICULAR', 'blue', 'orange', altitude), 8 ),
            ( FollowLineDirective('blue', speed = 0.8), 8 )
            ]
            algCycles = 6
            
            end = [
            ( OrientLineDirective('PARALLEL', 'green', 'orange', altitude ), 4 ),
            ( LandDirective(), 1)
            ]

            error = (ReturnToColorDirective('orange'), 10)
            
            self.MachineSwitch( init, alg, algCycles, end, error, AUTO_CIRCLE_MACHINE)


        elif key == ord('a'):

            self.moveTime = 0.0
            self.waitTime = 0.0

            alg = [
            ( PIDHoverColorDirective('orange', self.settingsPath), 10),
            ( PIDOrientLineDirective( 'PARALLEL', 'green', 'orange', self.settingsPath ), 10),
            ( SetCameraDirective("FRONT"), 1 ), 
            ( IdleDirective("Pause for setting camera"), 25 ),
            ( CapturePhotoDirective(self.droneRecordPath), 1 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective("Pause for setting camera"), 15 ),
            ( PIDOrientLineDirective( 'PERPENDICULAR', 'blue', 'orange', self.settingsPath ), 4),
            ( FollowLineDirective('blue'), 14 )
            ]
            
            algCycles = 6

            end = [
            ( PIDOrientLineDirective( 'PARALLEL', 'green', 'orange', self.settingsPath ), 4),
            ( LandDirective(), 1)
            ]

            error = (ReturnToColorDirective('orange', speedModifier = 0.4), 10)

            self.MachineSwitch(None, alg, algCycles, None , error, PID_AUTO_CIRCLE_MACHINE)
    
        elif key == ord('p'):

            self.moveTime = 0.0
            self.waitTime = 0.0
            

            #pidAlg = PIDOrientLineDirective( 'PARALLEL', 'green', 'orange', self.settingsPath)
            #pidAlg = PIDOrientLineDirective( 'PERPENDICULAR', 'blue', 'orange', self.settingsPath)
            pidAlg = PIDHoverColorDirective('orange',self.settingsPath)
            alg = [
            (pidAlg, 10)
            ]
            p,i,d = pidAlg.GetSettings(self.settingsPath)
            pidAlg.pid.ResetPID(p,i,d)

            algCycles = -1
            error = (ReturnToColorDirective('orange', speedModifier = 0.5), 10)
            self.MachineSwitch( None, alg, algCycles, None, error, PID_HOVER_ORANGE_MACHINE )
        
        # just contains a machine to test any particular directive
        elif key == ord('b'):
            
            self.moveTime = 0.04
            self.waitTime = 0.14

            #testalg = ( OrientLineDirective( 'PARALLEL', 'green', 'orange', 700 ), 10 )
            #testalg = ( PIDOrientLineDirective( 'PERPENDICULAR', 'blue', 'orange', self.settingsPath ), 4)
            #testalg = ( FollowLineDirective('blue', speed = 0.8), 14 )
            testalg = (BinaryTestDirective(), 4)

            alg = [
            testalg
            ]
            algCycles = -1

            self.MachineSwitch( None, alg, algCycles, None, None, TEST_MACHINE)


    # Taking in some machine's definition of states and a string name,
    # provides a "switch" for loading and removing the machines that
    # drone master uses to control the drone
    def MachineSwitch(self, newMachineInit, newMachineAlg,
    newMachineEnd, newMachineErr, newMachineAlgCycles, newMachineName):
        
        # pause the current state
        self.controller.SetCommand(0,0,0,0)

        oldMachine = self.currMachine
        # if the current machine is toggled again with the same machine,
        # remove the machine and return back to the idle 
        if oldMachine == newMachineName:
            self.currMachine = None
        
        # Otherwise, just switch to the machine
        else:
            self.stateMachine.DefineMachine(newMachineInit, newMachineAlg,
            newMachineEnd, newMachineErr, newMachineAlgCycles)
            self.currMachine = newMachineName
        
        rospy.logwarn('======= Drone Master: Changing from the "' + str(oldMachine) +
        '" machine to the "' + str(self.currMachine) + '" machine =======')


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
            droneInstructions[2], droneInstructions[3], self.moveTime, self.waitTime)


    # this function will go a certain speed for a set amount of time
    def MoveFixedTime(self, xSpeed, ySpeed, yawSpeed, zSpeed, move_time, wait_time):
       
        if time.clock() < (self.startTimer+move_time) or wait_time == 0:
            xSetSpeed = xSpeed
            ySetSpeed = ySpeed
            yawSetSpeed = yawSpeed
            zSetSpeed = zSpeed
            rospy.logwarn("moving **********")

        else:
            xSetSpeed = 0
            ySetSpeed = 0
            yawSetSpeed = 0
            zSetSpeed = 0
            rospy.logwarn("waiting")
            if time.clock() > (self.startTimer + move_time + wait_time):
                rospy.logwarn("reset")
                self.startTimer=time.clock()
       
        self.controller.SetCommand(xSetSpeed, ySetSpeed, yawSetSpeed, zSetSpeed)

        # log info
        #self.logger.Log("cx: " + str(self.cx) + " cy: " + str(self.cy) +
        self.logger.Log(
        " altitude: " + str(self.flightInfo["altitude"]) +
        " yawSpeed: " + str(yawSetSpeed) + " zSpeed: " + str(zSetSpeed) )


    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        self.logger.Stop()



if __name__=='__main__':
    
    rospy.init_node('DroneMaster')
    master = DroneMaster()
    rospy.on_shutdown(master.ShutdownTasks)
    rospy.spin()

    

