#!usr/bin/env python

import rospy
from AbstractDroneDirective import *


# Will make the drone fly to some specified altitude
class ReachAltitudeDirective(AbstractDroneDirective):

    # sets up this directive
    # altitude is in mm
    def __init__(self, desiredAltitude, tolerance):
        self.desiredAltitude = desiredAltitude
        self.tolerance = tolerance
        self.moveTime = 0.20
        self.waitTime = 0.10


    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone has reached the desired altitude
    #   0 if the algorithm is still running and the drone hasn't reached the desired altitude
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        currAltitude = navdata["SVCLAltitude"][1]
        climbspeed = 1

        if currAltitude < self.desiredAltitude - self.tolerance:
            zVel = climbspeed
            directiveStatus = 0
            rospy.logwarn("Drone is increasing altitude; Current is " + str(currAltitude) + " mm, desired is: " + str(self.desiredAltitude)+ " mm")
        
        elif currAltitude > self.desiredAltitude + self.tolerance:
            zVel = climbspeed * -1
            directiveStatus = 0
            rospy.logwarn("Drone is decreasing altitude; Current is " + str(currAltitude) + " mm, desired is: " + str(self.desiredAltitude)+ " mm")

        else:
            zVel = 0
            directiveStatus = 1

        return directiveStatus, (0, 0, 0, zVel), image, (None,None), self.moveTime, self.waitTime, None

