#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import rospy
import os

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist      # for sending commands to the drone
from std_msgs.msg import Empty           # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):

    def __init__(self, name="default"):
        # Holds the current drone status
        self.status = -1

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
        
        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/ardrone/land',Empty, queue_size = 10)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty, queue_size = 10)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty, queue_size =10)
        
        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

        # Setup regular publishing of control packets
        self.command = Twist()
        #self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)
        self.commandTimer = None 

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

        # Setting name of controller, for logging purposes
        self.controllerName = name


    def ReceiveNavdata(self,navdata):
        # Although there is a lot of data in this packet, we're only interested in the state at the moment  
        self.status = navdata.state


    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        if(self.status == DroneStatus.Landed):
            self.pubTakeoff.publish(Empty())


    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())


    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset.publish(Empty())
    

    # explicitly switch to a camera
    # camera=0 is for front camera, camera=1 is for bottom camera
    def SwitchCamera(self, camera):
        #toggle the camera
        if camera != 0 and camera !=1:
            raise ValueError("Camera must be 0 or 1")
        os.system("rosservice call /ardrone/setcamchannel "+str(camera))


    # switch to the other camera
    def ToggleCamera(self):
        os.system("rosservice call /ardrone/togglecam")


    # set flattrim to re-calibrate its rotation estimates
    def FlatTrim(self):
        os.system("rosservice call /ardrone/flattrim")


    def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x  = pitch
        self.command.linear.y  = roll
        self.command.linear.z  = z_velocity
        self.command.angular.z = yaw_velocity

        """if self.commandTimer == None:
            ctbool = " None "
        else:
            ctbool = " Not None " 

        rospy.logwarn("setting command:" + ctbool + str(self.command.linear.x) + 
        str(self.command.linear.y) + str(self.command.linear.z) + str(self.command.angular.z))
        """

        if( self.command.linear.x == 0 and self.command.linear.y == 0 and
        self.command.linear.z == 0 and self.command.angular.z == 0 ):
            #self.SendCommand(None)
            self.pubCommand.publish(self.command)
            #rospy.logwarn("attempting to shut timer down")
            if self.commandTimer != None:
                #rospy.logwarn("SHUTTING DOWN TIMER")
                self.commandTimer.shutdown()
                self.commandTimer = None
        else:
            #rospy.logwarn("STARTING TIMER")
            self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand, True)


    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
            #rospy.logwarn(self.controllerName +" publishing " + str(self.command.linear.x) +
            #str(self.command.linear.y)+ str(self.command.linear.z) + str(self.command.angular.z) )

            self.pubCommand.publish(self.command)





