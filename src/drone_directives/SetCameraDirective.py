#!usr/bin/env python

import rospy
from AbstractDroneDirective import *
from drone_controller import BasicDroneController


# Will toggle the drone's camera to a specified camera
class SetCameraDirective(AbstractDroneDirective):

    # sets up this directive
    def __init__(self, camera):
        
        # camera=0 is for front camera, camera=1 is for bottom camera
        if camera == "FRONT":
            self.camera = 0
        elif camera == "BOTTOM":
            self.camera = 1
        else:
            raise Exception("Camera must be 0 (front) or 1 (bottom)")

        self.controller = BasicDroneController("Toggle Camera Directive")
    

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   1 if algorithm is finished and the drone's camera has been set
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        self.controller.SwitchCamera(self.camera)

        if self.camera == 0:
            cameraName = "front camera"
        else:
            cameraName = "bottom camera"

        rospy.logwarn("Set Drone Camera to: " + cameraName)

        return 1, (0, 0, 0, 0), image, (None,None), 0, 0, None

