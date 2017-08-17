#!usr/bin/env python

import rospy
from AbstractDroneDirective import *

# Will make the drone land 
class TestSlowDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self):
        pass

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
        
        speed = 0.06
        rospy.logwarn("Testing a speed of " + str(speed))

        return 1, (0, speed, 0, 0), image, (None,None)

