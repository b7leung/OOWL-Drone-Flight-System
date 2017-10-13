#!usr/bin/env python

import rospy
from AbstractDroneDirective import *
from drone_controller import BasicDroneController


# Will make the drone land 
class TakeoffDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self):
        
        self.controller = BasicDroneController("Takeoff Directive")
    

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone is taking off
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        self.controller.SendTakeoff()
        rospy.logwarn("Drone is taking off")

        return 1, (0, 0, 0, 0), image, (None,None), 0, 0, None

