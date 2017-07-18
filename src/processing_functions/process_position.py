#!/usr/bin/env python

import sys
import rospy

#load the controller that has drone controlling functions
from drone_controller import BasicDroneController

class DronePosition(object):
    def __init__(self):
        #create instance of BasicDroneControler
        self.controller=BasicDroneController()
        
        #send command to the drone to hover ontop of checkpoint
        self.controller.SetCommand(self.roll1, self.pitch1, self.yaw1, self.altitude1)

