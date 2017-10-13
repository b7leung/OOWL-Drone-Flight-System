#! usr/bin/env/python

import rospy
from AbstractDroneDirective import *
from drone_controller import BasicDroneController

#Will re-calibrate drone setting on a flat surface
#Should never be called if drone in take-off
class FlatTrimDirective(AbstractDroneDirective):

    def __init__(self):

        self.controller = BasicDroneController("Flat Trim Directive")

    def RetrieveNextInstruction(self,image,navdata):

        self.controller.FlatTrim()
        rospy.logwarn("Reset Flat Trim")

        return 1, (0, 0, 0, 0), image, (None,None),0,0, None
