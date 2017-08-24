#!/usr/bin/env python

import sys
import rospy

#load the controller that has drone controlling functions
from drone_controller import BasicDroneController
from processing_functions.process_video import ProcessVideo
from processing_functions.pid_controller import PIDController

class DronePosition(object):

    def __init__(self,image):

        self.pid = PIDController()
        self.process = ProcessVideo()

    #take input as bgr image, given orange is visible, will command to hover ontop of it
        def HoverOnCheckpoint(self,image):

            # calculating cx,cy,xspeed,yspeed
            orange_image = self.process.DetectColor(image,'orange')
            self.cx,self.cy = self.process.CenterofMass(orange_image)
            self.pid.UpdateDeltaTime()
            self.pid.SetPoint(orang_image)
            self.pid.UpdateError(self.cx,self.cy)
            self.pid.SetPIDTerm()
            xspeed, yspeed = self.pid.GetPIDValues()
            self.pid.DrawArrow(orange_image, xspeed, yspeed)
    

            return xspeed,yspeed

        def OrienttoLine(self,image):

            orange_image = self.process.DetectColor(image,'orange')
            self.cx,self.cy = self.process.CenterofMass(orange_image)
            blue_image = self.process.DetectColor(image,'blue')
            angle = self.process.ShowLine(blue_image)
            yawspeed = self.process.LineOrientation(blue_image,angle,5)

            return yawspeed

        def OrienttoObject(self,image):

            orange_image = self.process.DetectColor(image,'orange')
            self.cx,self.cy = self.process.CenterofMass(orange_image)
            green_image = self.process.DetectColor(image,'green')
            angle = self.process.ShowLine(green_image)
            yawspeed = self.process.ObjectOrientation(green_image,angle,5)

            return yawspeed
            
