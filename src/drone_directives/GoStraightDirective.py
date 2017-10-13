#!usr/bin/env python

import rospy
from AbstractDroneDirective import *
from drone_controller import BasicDroneController

# Simply tell the drone to go straight, in some direction
class GoStraightDirective(AbstractDroneDirective):


    # sets up this directive
    def __init__(self, direction, speed):

        
        if speed > 1 or speed < 0:
            raise Exception("Speed must be at least 0 and no more than 1")

        if direction == "GO_RIGHT":
            self.instructions = (speed*-1,0,0,0)

        elif direction == "GO_LEFT":
            self.instructions = (speed,0,0,0)

        elif direction == "GO_FORWARDS":
            self.instructions = (0,speed,0,0)

        elif direction == "GO_BACKWARDS":
            self.instructions = (0,speed * -1,0,0)

        elif direction == "GO_UP":
            self.instructions = (0,0,0,speed)

        elif direction == "GO_DOWN":
            self.instructions = (0,0,0,speed * -1)

        elif direction == "TURN_RIGHT":
            self.instructions = (0,0,speed*-1,0)

        elif direction == "TURN_LEFT":
            self.instructions = (0,0,speed,0)

        else:
            raise Exception("Direction not recognized")

        self.direction = direction
        self.moveTime = 0.20
        self.waitTime = 0.10


    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone is going the specified direction
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        rospy.logwarn("Drone is performing: " + self.direction)

        return 1, self.instructions, image, (None,None), self.moveTime, self.waitTime, None

