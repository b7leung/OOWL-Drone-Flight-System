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
        
        self.objectName = "NASA Airplane"
        self.startingAngle = 0
        
        # backpack: 120/-55/95
        # +100 for big objects, +50 for shorter objects. (Modifies how close drone is to object; smaller # > closer)
        # +10 for very small objects.
        self.yOffset = 0
        # -30 for big objects, -15 for shorter objects. (Modifies how close drone is to object; larger # > closer)
        self.ySizeOffset = -30
        # +75 for tall objects or using cube, 0 for shorter objects. (Modifies how high the drone should fly; smaller # > lower
        self.zOffset = 25


        # Seting up a timestamped folder inside Flight_Info that will have the pictures & log of this flight
        self.droneRecordPath= (expanduser("~")+"/drone_workspace/src/ardrone_lab/src/Flight_Info/"
        + datetime.datetime.now().strftime("%m-%d-%Y__%H:%M:%S, %A")+"_Flight_" + self.objectName + "/")
        if not os.path.exists(self.droneRecordPath):
            os.makedirs(self.droneRecordPath)
        #self.logger = Logger(self.droneRecordPath, "AR Drone Flight")
        #self.logger.Start()

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
        self.maxHeight = 2530
        self.enableEmergency = False
        self.emergency = False
        self.captureRound = 0.5
        self.oldBattery = -1
        self.photoDirective = None


    # Each state machine that drone mastercan use is defined here;
    # When key is pressed, define the machine to be used and switch over to it.
    # Machines are defined as array of tuples. Each tuple represents a state's directive and duration
    # in the format (directive, stateduration, errorDirective). 
    # ErrorDirectives are optional, so it can be just in the format
    # (directive, stateduration);
    # directive & errorDirective must subclass AbstractDroneDirective.
    def KeyListener(self):

        key=cv2.waitKey(1) & 0xFF
        
        # hover over orange
        if key == ord('1'):
            
            self.moveTime = 0.04
            self.waitTime = 0

            testalg = ( PIDHoverColorDirective2("orange"), 6)

            algCycles = -1

            alg = [testalg]

            self.MachineSwitch( None, alg, algCycles, None, HOVER_ORANGE_MACHINE)


        # take picture
        if key == ord('3'):

            pictureName = self.pictureManager.Capture(self.cv_image)
            rospy.logwarn("Saved picture")

        # toggle camera
        elif key == ord('c'):

            self.controller.ToggleCamera()
            rospy.logwarn("Toggled Camera")

        # land (32 => spacebar)
        elif key == 32:

            self.controller.SendLand()
            rospy.logwarn("***______--______-_***Landing Drone***_-______--_____***")
            # if there is a photo directive running, save pictures just in case
            self.SaveCachePictures()

        # save all pictures in cache
        elif key == ord('s'):
            self.SaveCachePictures()

        elif key == ord('q') or key == ord('t'):

            # main algorithm components
            self.moveTime = 0.20
            self.waitTime = 0.10
            flightAltitude = 1360 + self.zOffset
            objectAltitude = 1360 + self.zOffset

            angles = 8
            # ~30 for big objects, ~ for small objects (can-sized)
            heightTolerance = 32
            orangePlatformErrHoriz= (ReturnToColorDirective('orange', "blue", speedModifier = 0.85), 4)
            orangePlatformErrParallel= (ReturnToColorDirective('orange', "pink", speedModifier = 0.85), 4)
            blueLineErr = (ReturnToLineDirective('blue', speedModifier = 0.85), 6)
            self.SaveCachePictures()
            self.photoDirective = CapturePhotoDirective(self.droneRecordPath, 30, 0.014, self.objectName, angles, objectAltitude, self.startingAngle)

            # 0.018

            alg = [
            ( OrientLineDirective( 'PARALLEL', 'pink', 'orange', flightAltitude, heightTolerance, self.yOffset, self.ySizeOffset), 1, orangePlatformErrParallel),
            ( SetCameraDirective("FRONT"), 1 ), ( IdleDirective("Pause for setting camera to front"), 25 ),
            ( self.photoDirective, 1 ),
            ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective("Pause for setting camera to bottom"), 25 ),
            ( OrientLineDirective('PERPENDICULAR', 'blue', 'orange', flightAltitude), 5, orangePlatformErrHoriz),
            ( FollowLineDirective('blue', speed = 0.09), 6, blueLineErr )
            ]
            
            # land on the 8th angle
            end = [
            ( OrientLineDirective( 'PARALLEL', 'pink', 'orange', flightAltitude, heightTolerance, self.yOffset, self.ySizeOffset), 1, orangePlatformErrParallel),
            ( SetCameraDirective("FRONT"), 1 ), ( IdleDirective("Pause for setting camera to bottom"), 25 ),
            ( self.photoDirective, 1 ),
            ( LandDirective(), 1), ( IdleDirective("Pause for land"), 25 ),
            ( self.photoDirective, 1, None, "SavePhotos")
            ]


            if key == ord('q'):
                
                #doesn't auto takeoff

                self.MachineSwitch( None, alg, angles-1, end, AUTO_CIRCLE_MACHINE)


            if key == ord('t'):

                # does the entire circle algorithm, in order; takes off by itself

                init = [
                ( SetupDirective(), 1), ( IdleDirective("Pause for setup"), 10 ),
                ( FlatTrimDirective(), 1), ( IdleDirective("Pause for flat trim"), 10 ),
                ( SetCameraDirective("BOTTOM"), 1 ), ( IdleDirective(" Pause for seting camera"), 10 ),
                ( TakeoffDirective(), 1), ( IdleDirective("Pause for takeoff"), 120 ),
                ( ReturnToOriginDirective('orange',50), 7 ),
                ( FindPlatformAltitudeDirective('orange', flightAltitude + 200), 5),
                #( ReachAltitudeDirective(flightAltitude, 85), 2)
                ]

                self.MachineSwitch( init, alg, angles-1, end, AUTO_CIRCLE_MACHINE)

        
        # just contains a machine to test any particular directive
        elif key == ord('b'):
            
            self.moveTime = 0.04
            self.waitTime = 0.14

            error = (ReturnToLineDirective('blue', speedModifier = 0.4), 10)
            blueLineErr = (ReturnToLineDirective('blue'), 10)
            #orangePlatformErr = (ReturnToColorDirective('orange'), 10)
            orangePlatformErr = None
            orangePlatformErrHoriz= (ReturnToColorDirective('orange', "blue"), 10)

            #testalg = ( OrientLineDirective( 'PARALLEL', 'pink', 'orange', 1360, 150, self.yOffset, self.ySizeOffset), 1, orangePlatformErr)
            #testalg = ( PIDOrientLineDirective( 'PERPENDICULAR', 'blue', 'orange', self.settingsPath ), 4, error)
            #testalg = ( FollowLineDirective('blue', speed = 0.25), 6, blueLineErr)
            #testalg = ( OrientLineDirective('PERPENDICULAR', 'blue', 'orange', 700), 8, orangePlatformErrHoriz)
            #testalg = ( CapturePhotoDirective(self.droneRecordPath, 10, 0.3), 1 )
            testalg = ( MultiCenterTestDirective("orange"), 6)

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
        currHeightReg = self.flightInfo["altitude"][1]
        if currHeightReg == '?':
            currHeightReg = 0

        currHeightCalc = self.flightInfo["SVCLAltitude"][1]
        
        if currHeightCalc != -1:
            height = currHeightCalc
        else:
            height = currHeightReg

        #rospy.logwarn( "reg: " + str(currHeightReg) + " Calc: " + str(currHeightCalc) +
        #" avg: " + str(height))


        if self.enableEmergency and height > self.maxHeight:
            self.controller.SendLand()
            self.emergency = True

        if self.emergency:
            rospy.logwarn("***** EMERGENCY LANDING: DRONE'S ALTITUDE IS " + str(height) +" mm; MAX IS " +
            str(self.maxHeight) + " mm *****")
            rospy.logwarn("***______--______-_***Landing Drone***_-______--_____***")
            # if there is a photo directive running, save pictures just in case
            if self.photoDirective != None:
                self.photoDirective.SavePhotos(None,None)
                self.photoDirective = None

  
        # If no machine is loaded, then drone master does nothing 
        # (so that the drone may be controlled with the keyboard)
        if self.currMachine == None:

            pass
                
        else:
            # retrieving the instructions for whatever state the machine is in 
            # and commanding the drone to move accordingly
            droneInstructions, segImage, moveTime, waitTime = self.stateMachine.GetUpdate(self.cv_image, self.flightInfo)
            self.cv_image = segImage
            #self.MoveFixedTime(droneInstructions[0], droneInstructions[1],
            #droneInstructions[2], droneInstructions[3], moveTime, waitTime)
        
        # draws battery display and height for info Window

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

            if self.flightInfo["batteryPercent"][1] != self.oldBattery:
                self.oldBattery = self.flightInfo["batteryPercent"][1] 
                self.info = np.zeros((70,100,3), np.uint8)
                cv2.putText(self.info, batteryPercent,
                (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1, color,1,cv2.LINE_AA)

        #cv2.putText(self.info, str(int(height)) + " mm",
        #(50,120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1,cv2.LINE_AA)



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
        #self.logger.Log(
        #" altitude: " + str(self.flightInfo["altitude"]) +
        #" yawSpeed: " + str(yawSetSpeed) + " zSpeed: " + str(zSetSpeed) )


    # if there is a photo directive running, save pictures
    def SaveCachePictures(self):
        rospy.logwarn("saving cache pictures")
        if self.photoDirective != None:
            self.photoDirective.SavePhotos(None,None)
            self.photoDirective = None
        else:
            rospy.logwarn("none")


    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        self.logger.Stop()


if __name__=='__main__':
    
    rospy.init_node('DroneMaster')
    master = DroneMaster()
    rospy.on_shutdown(master.ShutdownTasks)
    rospy.spin()

