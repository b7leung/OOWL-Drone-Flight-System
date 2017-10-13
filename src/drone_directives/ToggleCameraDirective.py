#!usr/bin/env python

import rospy
from AbstractDroneDirective import *
from drone_controller import BasicDroneController


# Will toggle the drone's camera
class ToggleCameraDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self):
        
        self.controller = BasicDroneController("Toggle Camera Directive")
    

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone's camera has been toggled
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        self.controller.ToggleCamera()
        rospy.logwarn("Toggled Drone Camera")

        return 1, (0, 0, 0, 0), image, (None,None), 0, 0, None

