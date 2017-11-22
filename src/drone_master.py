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
HOVER_ORANGE_MACHINE = "hover_orange"
FACE_OBJECT_MACHINE = 'face_object'
CAPTURE_PHOTO_MACHINE = 'capture_photo'
FIX_TO_BLUE_LINE_MACHINE = "fix_to_blue"
FOLLOW_BLUE_LINE_MACHINE = "follow_blue"
AUTO_CIRCLE_MACHINE= "auto_circle"
PID_AUTO_CIRCLE_MACHINE= "pid_auto_circle"
PID_HOVER_ORANGE_MACHINE = 'pid_hover_orange'
SELF_CORRECTING_TAKEOFF_MACHINE = "self_correcting_takeoff"
TEST_MACHINE = 'test_machine'

# Drone Video Dimensions: Height = 360; Width = 640
# A class that has access to the drone video feed and navdata. It uses this information to feed into
# any one of its defined state machines, which will tell this class how to control the drone.
# The particular state machine to use can be changed at run time.
class DroneMaster(DroneVideo, FlightstatsReceiver):


    def __init__(self):

        # getting access to elements in DroneVideo and FlightstatsReciever
        super(DroneMaster,self).__init__()
        
        self.objectName = "GREEN_SCREEN"

        # Seting up a timestamped folder inside Flight_Info that will have the pictures & log of this flight
        self.droneRecordPath= (expanduser("~")+"/drone_workspace/src/ardrone_lab/src/Flight_Info/"
        + datetime.datetime.now().strftime("%m-%d-%Y__%H:%M:%S, %A")+"_Flight_" + self.objectName + "/")
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
        self.controller = BasicDroneController("TraceCircle")
        self.startTimer = time.clock()
        # max height of drone, in mm; any higher and the drone will auto-land
        self.maxHeight = 2000
        self.emergency = False
        self.captureRound = 0.5


    # Each state machine that drone mastercan use is defined here;
    # When key is pressed, define the machine to be used and switch over to it.
    # Machines are defined as array of tuples. Each tuple represents a state's directive and duration
    # in the format (directive, stateduration, errorDirective). 
    # ErrorDirectives are optional, so it can be just in the format
    # (directive, stateduration);
    # directive & errorDirective must subclass AbstractDroneDirective.
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
            ]
            self.MachineSwitch( init, None, 0, None, SELF_CORRECTING_TAKEOFF_MACHINE)

        elif key == ord('1'):

            self.moveTime = 0.15
            self.waitTime = 0.08
            alg = [(HoverColorDirective('orange', 700), 0)]
            algCycles = -1
            self.MachineSwitch( None , alg, algCycles, None, HOVER_ORANGE_MACHINE )
            
        elif key == ord('2'):

            self.moveTime = 0.22
            self.waitTime = 0.12
            alg = [( OrientLineDirective( 'PARALLEL', 'green', 'orange', 700), 0 )]
            algCycles = -1
            self.MachineSwitch( None, alg, algCycles, None, FACE_OBJECT_MACHINE)

        elif key == ord('3'):

            pictureName = self.pictureManager.Capture(self.cv_image)
            rospy.logwarn("Saved picture as " + pictureName)

        elif key == ord('4'):

            self.moveTime = 0.15
            self.waitTime = 0.08
            alg = [ ( OrientLineDirective('PERPENDICULAR', 'blue', 'orange', 700), 0 ) ]
            algCycles = -1
            self.MachineSwitch( None, alg, algCycles, None, FIX_TO_BLUE_LINE_MACHINE)

        elif key == ord('5'):
            
            self.moveTime = 0.15
            self.waitTime = 0.08
            alg = [(FollowLineDirective('blue'), 0)]
            algCycles = -1
            self.MachineSwitch( None, alg, algCycles, None, FOLLOW_BLUE_LINE_MACHINE)

        elif key == ord('f'):
           
            self.moveTime = 0.20
            self.waitTime = 0.10
            flightAltitude = 1300
            objectAltitude = 1300
            self.captureRound+=0.5

            orangePlatformErr = (ReturnToColorDirective('orange'), 4)
            
            angles = 8
            alg = [
            ( OrientLineDirective( 'PARALLEL', 'green', 'orange', flightAltitude ), 10, orangePlatformErr ),
            ( SetCameraDirective("FRONT"), 1 ), ( IdleDirective("Pause for setting camera"), 25 ),
            ( CapturePhotoDirective2(self.droneRecordPath, 20, 0.07, self.objectName, angles, self.captureRound, objectAltitude), 1 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective("Pause for setting camera"), 15 )
            ]
            
            
            self.MachineSwitch( None, alg, angles, None, AUTO_CIRCLE_MACHINE)

        elif key == ord('s'):

            # does the entire circle algorithm, in order.

            self.moveTime = 0.20
            self.waitTime = 0.10
            flightAltitude = 1380 
            objectAltitude = 1380
                        
            init = [
            ( SetupDirective(), 1), ( IdleDirective("Pause for setup"), 10 ),
            ( FlatTrimDirective(), 1), ( IdleDirective("Pause for flat trim"), 10 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective(" Pause for seting camera"), 10 ),
            ( TakeoffDirective(), 1), ( IdleDirective("Pause for takeoff"), 120 ),
            ( ReturnToOriginDirective('orange',50), 7 ),
            ( FindPlatformAltitudeDirective('orange', flightAltitude + 200), 5)
            ]
            
            orangePlatformErrHoriz= (ReturnToColorDirective('orange', "blue", speedModifier = 0.85), 4)
            orangePlatformErrParallel= (ReturnToColorDirective('orange', "pink", speedModifier = 0.85), 4)
            blueLineErr = (ReturnToLineDirective('blue', speedModifier = 0.85), 6)

            angles = 8
            #30
            photoDirective = CapturePhotoDirective(self.droneRecordPath, 30, 0.04, self.objectName, angles, objectAltitude)

            alg = [
            ( OrientLineDirective( 'PARALLEL', 'pink', 'orange', flightAltitude ), 2, orangePlatformErrParallel ),
            ( SetCameraDirective("FRONT"), 1 ), ( IdleDirective("Pause for setting camera"), 25 ),
            ( photoDirective, 1 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective("Pause for setting camera"), 25 ),
            ( ResumePlatformDirective('orange', speedModifier = 0), 2),
            ( OrientLineDirective('PERPENDICULAR', 'blue', 'orange', flightAltitude), 5, orangePlatformErrHoriz ),
            ( FollowLineDirective('blue', speed = 0.09), 6, blueLineErr )
            ]
            
            # land on the 8th angle
            end = [
            ( OrientLineDirective( 'PARALLEL', 'pink', 'orange', flightAltitude ), 3, orangePlatformErrParallel ),
            ( SetCameraDirective("FRONT"), 1 ), ( IdleDirective("Pause for setting camera"), 25 ),
            ( photoDirective, 1 ),
            ( SetCameraDirective("BOTTOM"), 1 ),
            ( LandDirective(), 1)
            ]

            self.MachineSwitch( None, alg, angles-1, end, AUTO_CIRCLE_MACHINE)

        elif key == ord('a'):

            self.moveTime = 0.1
            self.waitTime = 0.01

            orangePlatformErr= (ReturnToColorDirective('orange', speedModifier = 0.4), 10)

            alg = [
            ( PIDHoverColorDirective('orange', self.settingsPath), 10, orangePlatformErr),
            ( PIDOrientLineDirective( 'PARALLEL', 'green', 'orange', self.settingsPath ), 10, orangePlatformErr),
            ( SetCameraDirective("FRONT"), 1 ), 
            ( IdleDirective("Pause for setting camera"), 25 ),
            ( CapturePhotoDirective(self.droneRecordPath), 1 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective("Pause for setting camera"), 15 ),
            ( PIDOrientLineDirective( 'PERPENDICULAR', 'blue', 'orange', self.settingsPath ), 4, orangePlatformErr),
            ( FollowLineDirective('blue'), 14 )
            ]
            algCycles = 6

            end = [
            ( PIDOrientLineDirective( 'PARALLEL', 'green', 'orange', self.settingsPath ), 4, orangePlatformErr),
            ( LandDirective(), 1)
            ]

            self.MachineSwitch(None, alg, algCycles, None , PID_AUTO_CIRCLE_MACHINE)
    
        elif key == ord('p'):

            
            #pidAlg = PIDOrientLineDirective( 'PARALLEL', 'green', 'orange', self.settingsPath)
            #pidAlg = PIDOrientLineDirective( 'PERPENDICULAR', 'blue', 'orange', self.settingsPath)
            pidAlg = PIDHoverColorDirective('orange',self.settingsPath)

            orangePlatformError = (ReturnToColorDirective('orange', speedModifier = 0.5), 10)

            alg = [(pidAlg, 10, orangePlatformError)]

            p,i,d = pidAlg.GetSettings(self.settingsPath)
            pidAlg.pid.ResetPID(p,i,d)

            algCycles = -1
            self.MachineSwitch( None, alg, algCycles, None, PID_HOVER_ORANGE_MACHINE )
        
        # just contains a machine to test any particular directive
        elif key == ord('b'):
            
            self.moveTime = 0.04
            self.waitTime = 0.14

            error = (ReturnToLineDirective('blue', speedModifier = 0.4), 10)
            blueLineErr = (ReturnToLineDirective('blue'), 10)
            #orangePlatformErr = (ReturnToColorDirective('orange'), 10)
            orangePlatformErr = None
            orangePlatformErrHoriz= (ReturnToColorDirective('orange', "blue"), 10)

            #testalg = ( OrientLineDirective( 'PARALLEL', 'pink', 'orange', 500 ), 10, orangePlatformErr )
            #testalg = ( PIDOrientLineDirective( 'PERPENDICULAR', 'blue', 'orange', self.settingsPath ), 4, error)
            #testalg = ( FollowLineDirective('blue', speed = 0.25), 6, blueLineErr)
            testalg = ( OrientLineDirective('PERPENDICULAR', 'blue', 'orange', 700), 8, orangePlatformErrHoriz)
            #testalg = ( CapturePhotoDirective(self.droneRecordPath, 10, 0.3), 1 )
            #testalg = ( MultiCenterTestDirective("orange"), 6)

            algCycles = -1

            alg = [testalg]

            self.MachineSwitch( None, alg, algCycles, None, TEST_MACHINE)


    # Taking in some machine's definition of states and a string name,
    # provides a "switch" for loading and removing the machines that
    # drone master uses to control the drone
    def MachineSwitch(self, newMachineInit, newMachineAlg,
     newMachineAlgCycles, newMachineEnd, newMachineName):
        
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
            newMachineAlgCycles, newMachineEnd, self )
            self.currMachine = newMachineName
        
        rospy.logwarn('======= Drone Master: Changing from the "' + str(oldMachine) +
        '" machine to the "' + str(self.currMachine) + '" machine =======')


    # This is called every time a frame (in self.cv_image) is updated.
    # Runs an iteration of the current state machine to get the next set of instructions, depending on the 
    # machine's current state.
    def ReceivedVideo(self):
        
        # checks altitude; if it is higher than allowed, then drone will land
        currHeight = self.flightInfo["altitude"][1]
        if currHeight != "?" and currHeight > self.maxHeight:
            self.controller.SendLand()
            self.emergency = True

        if self.emergency:
            rospy.logwarn("***** EMERGENCY LANDING: DRONE'S ALTITUDE IS " + str(currHeight) +" mm; MAX IS " +
            str(self.maxHeight) + " mm *****")
  
        # If no machine is loaded, then drone master does nothing 
        # (so that the drone may be controlled with the keyboard)
        if self.currMachine == None:

            pass
                
        else:
            # retrieving the instructions for whatever state the machine is in 
            # and commanding the drone to move accordingly
            droneInstructions, segImage, moveTime, waitTime = self.stateMachine.GetUpdate(self.cv_image, self.flightInfo)
            self.cv_image = segImage
            self.MoveFixedTime(droneInstructions[0], droneInstructions[1],
            droneInstructions[2], droneInstructions[3], moveTime, waitTime)
        
        # draws battery display
        color = (255,255,255)
        if self.flightInfo["batteryPercent"][1] != "?":

            batteryPercent = int(self.flightInfo["batteryPercent"][1])

            sum = int(batteryPercent * .01 * (255+255))
            if sum > 255:
                base = 255
                overflow = sum - 255
            else:       
                base = sum
                overflow = 0
            
            green = base
            red = 255 - overflow

            color = (0, green, red)

            batteryPercent = str(batteryPercent) + "%"

        else:
            batteryPercent = str((self.flightInfo["batteryPercent"][1]))+"%"

        cv2.putText(self.cv_image, batteryPercent,
        (560,345), cv2.FONT_HERSHEY_SIMPLEX, 1, color,1,cv2.LINE_AA)


    # this function will go a certain speed for a set amount of time, then rest for wait_time # of cycles
    def MoveFixedTime(self, xSpeed, ySpeed, yawSpeed, zSpeed, move_time, wait_time):
       
        # Moving
        if time.clock() < (self.startTimer+move_time) or wait_time == 0:
            xSetSpeed = xSpeed
            ySetSpeed = ySpeed
            yawSetSpeed = yawSpeed
            zSetSpeed = zSpeed

        # Waiting
        else:
            xSetSpeed = 0
            ySetSpeed = 0
            yawSetSpeed = 0
            zSetSpeed = 0
            # Resetting timer, so that drone moves again
            if time.clock() > (self.startTimer + move_time + wait_time):
                self.startTimer=time.clock()
       
        self.controller.SetCommand(xSetSpeed, ySetSpeed, yawSetSpeed, zSetSpeed)

        # logs info
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

