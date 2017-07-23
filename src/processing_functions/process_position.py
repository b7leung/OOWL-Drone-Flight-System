#!/usr/bin/env python

import sys
import rospy

#load the controller that has drone controlling functions
from drone_controller import BasicDroneController

class DronePosition(object):
    #take input as bgr image, given orange is visible, will command to hover ontop of it
    def HoverOnOrange(self,image):
        # calculating cx,cy,xspeed,yspeed
        orange_image=self.process.DetectColor(image,'orange')
        image=orange_image
        self.cx,self.cy=self.process.CenterofMass(orange_image)
        xspeed,yspeed=self.process.ApproximateSpeed(orange_image,self.cx,self.cy)

        return xspeed,yspeed

    def FollowBlue(self,image):
        blue_image=self.process.DetectColor(image,'blue')
        image=blue_image
        x0,y0,angle=self.process.ShowLine(blue_image)
        cx,cy=self.process.CenterofMass(blue_image)


    
