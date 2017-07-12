!#/usr/bin/env python

import sys
import rospy

#load the controller that has drone controlling functions
from drone_controller import BasicDroneController

class DronePosition(object):
    def __init__(self):
        #create instance of BasicDroneControler
        self.controller=BasicDroneController()
        
        self.pitch=0
        self.roll=0
        self.yaw=0
        self.altitude=0

    def DroneHover(self, xspeed, yspeed):
        #yspeed>0 means checkpoint is on top and drone should pitch forward
        #xspeed>0 means checkpoint is on the left and drone should roll left
        self.pitch=yspeed
        self.roll=xspeed

        #send command to the drone to hover ontop of checkpoint
        self.controller.SetCommand(self.roll, self.pitch, self.yaw, self.altitude)

