#!/usr/bin/env python

import rospy
from AbstractDroneDirective import *

# Describes the direction for the drone to stay still
class IdleDirective(AbstractDroneDirective):
    
    # sets up this directive
    def __init__(self):
        pass

    # Given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 indicating that the algorithm is finished and the drone is idle
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        rospy.logwarn("Drone is idle")

        return 1, (0, 0, 0, 0), image 

