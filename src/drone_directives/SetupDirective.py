#!usr/bin/env python

import rospy
from AbstractDroneDirective import *
from drone_controller import BasicDroneController


# Ensures that the drone is ready to fly
# By checking if it is in emergency mode
class SetupDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self):
        
        self.controller = BasicDroneController("Setup Directive") 


    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone is now ready to fly
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        state = navdata["state"][1]
        # Drone is currently in emergency mode, unset it
        if state == 0:
            self.controller.SendEmergency()

        rospy.logwarn("Drone is set up to fly")

        return 1, (0, 0, 0, 0), image, (None,None), 0, 0, None

