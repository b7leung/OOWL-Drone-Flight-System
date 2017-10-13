#!usr/bin/env python

import rospy
from AbstractDroneDirective import *


# Keeps the drone idle (cannot be controlled by any other controller, such as
# keyboard controller)
class IdleDirective(AbstractDroneDirective):


    # sets up this directive
    # reason is an optional description that states why drone was set to idle
    def __init__(self, reason = None):

        if reason == None:
            self.reason = "Not Given"
        else:
            self.reason = reason
        

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 as long as the drone is idle
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        rospy.logwarn("Drone is idle; Reason: " + self.reason)
        return 1, (0, 0, 0, 0), image, (None, None), 0, 0, None

