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

    def PIDController(self, image, cx, cy, time):

        numRows,numCols,channels=image.shape
        
        centerx=numCols/2
        centery=numRows/2

        width=60
        height=60
        
        xLower=centerx-width
        xUpper=centerx+width
        yLower=centery-height
        yUpper=centery+height

        dt=time_prevTime
        prevTime=time

        integralMax=100
        integralMin=-100

        xDerivator=0
        yDerivator=0

        K_p=0.5
        K_i=0.1
        K_d=0.1

        xError = (centerx-cx)/float(centerx)
        yError = (centery-cy)/float(centery)

        x_pValue = K_p * xError
        y_pValue = K_p * yError
        
        xIntegral += xError
        yIntegral += yError

        if xIntegral > integralMax:
            xIntegral = integralMax
        elif yIntegral > integralMax:
            yIntegral = integralMax
        elif yIntegral<integralMin:
            yIntegral = integralMin
        elif yIntegral < integralMin:
            yIntegral = integralMin

        x_iValue = K_i * xIntegral
        y_iValue = K_i * yIntegral

        xDerivative = xError - xDerivator
        yDerivative = yError - yDerivator

        xDerivator = xError
        yDerivator = yError

        x_dValue = K_d * xDerivative
        y_dValue = K_d * yDerivative
        
        x_PID = x_pValue + x_iValue
        y_PID = y_pValue + y_iValue


