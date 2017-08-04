#!/usr/bin/env python

import rospy
import numpy as np

 class PIDController(object):

    def __init__(self,Kp=0.5, Ki=0.0, Kd=0.0, moveTime = 0.1, waitTime = 0.04):
        
        self.xP = Kp/320
        self.xI = Ki/320
        self.xD = Kd/320
        
        self.yP = Kp/180
        self.yI = Ki/180
        self.yD = Kd/180
        
        
        self.xDerivator = 0.0
        self.yDerivator = 0.0
        self.xIntegrator = 0.0
        self.yIntegrator = 0.0

        self.xError = 0.0
        self.yError = 0.0

        self.dt = moveTime + waitTime
        self.oldTime = rospy.Time.now()

        self.InitializeFilter()

    #Calculation for the P_term
    #Linear relationship between Error and output
    def SetpTerm(self):
        
        self.x_pTerm = self.xP * self.xError
        self.y_pTerm = self.yP * self.yError
        
    #Calculation for the I_term
    #Gets the accumulation of error over time to assist the P_term in pushing the drone
    def SetiTerm(self):

        self.xIntegrator = self.xFiltered * dt
        self.yIntegrator = self.yFiltered * dt

        self.xIntegral += self.xIntegrator
        self.yIntegral += self.yIntegrator

        self.x_iTerm = self.xI * self.xIntegral
        self.y_iTerm = self.yI * self.yIntegral

    #Calculation for the D_term
    #Computes the rate of change of our Error to compensate for overshooting the command
    def SetdTerm(self):

        self.xTemp = self.xFiltered - self.xDerivator
        self.yTemp = self.yFiltered - self.yDerivator
        
        if self.dt > 0.0:
            self.xDerivative = self.xTemp/self.dt
            self.yDerivative = self.yTemp/self.dt
        else:
            self.xDerivative = 0.0
            self.yDerivative = 0.0

        self.x_dTerm = self.xD * self.xDerivative
        self.y_dTerm = self.yD * self.yDerivative

        self.xDerivator = self.xFiltered
        self.yDerivator = self.yFiltered

    #Sum up the three P,I,D terms to get the output Command
    #Return xspeed for roll and yspeed for pitch
    def GetPIDValues(self):

        xPID = self.x_pTerm + self.x_iTerm + self.x_dTerm
        yPID = self.y_pTerm + self.y_iTerm + self.y_dTerm
        
        return xPID,yPID

        
    #Compute the desired SetPoint for the Drone - the center of the image
    def SetPoint(self,image):
        
        self.numRows, self.numCols, self.channels = image.shape
        self.centerx = self.numCols/2
        self.centery = self.numRows/2

    #Calculate Error as a function of object's distance from desired setpoint (the center)
    #Perform a LowPass Filter on the Error Values for the Integral and Derivative Terms
    def UpdateError(self,cx,cy):
        
        self.xError = self.centerx - cx
        self.yError = self.centery - cy
        self.xFiltered, self.yFiltered = self.LowPassFilter(self.xError, self.yError)

   #Set the Coefficient to an appropriate value based on trial and Error 
    def SetPID(self,Kp,Ki,Kd):
        
        self.xP = Kp/320
        self.xI = Ki/320
        self.xD = Kd/320
        
        self.yP = Kp/180
        self.yI = Ki/180
        self.yD = Kd/180

    def UpdateDeltaTime(self):
        
        self.timeNow = rospy.Time.now()
        self.dt = self.timeNow - self.oldTime
        self.oldTime = self.timeNow
    
    #Filter coefficients copied from github.com/raultrom/ardrone_velocity
    #https://github.com/raultron/ardrone_velocity/blob/master/filtervelocity.cpp
    #August 3rd 2017
    #LowPassFilter
    def InitializeFilter(self):

        self.coef = np.array([0.00498902, 0.00567655, 0.00768429, 0.01092849, 0.01526534,
                0.02049766, 0.02638413, 0.03265082, 0.03900430, 0.04514569,
                0.05078516, 0.05565588, 0.05952694, 0.06221459, 0.06359114,
                0.06359114, 0.06221459, 0.05952694, 0.05565588, 0.05078516,
                0.04514569, 0.03900430, 0.03265082, 0.02638413, 0.02049766,
                0.01526534, 0.01092849, 0.00768429, 0.00567655, 0.00498902])
        
        self.filterSize = coef.size

        self.xBuffer = np.zeros(filterSize)
        self.yBuffer = np.zeros(filterSize)
    
    #Perform a LowPassFilter on Error values to remove noise and high-freq values
    #return filtered error values
    def LowPassFilter(self,xError,yError):
    
        self.xBuffer[0] = xError
        self.yBuffer[0] = yError

        for i in np.arange(self.filterSize):
            xFiltered += self.xBuffer[i] * self.coef[i]
            yFiltered += self.yBuffer[i] * self.coef[i]

        for i in np.arange(self.filterSize-1,0,-1):
            self.xBuffer[i] = self.xBuffer[i-1]
            self.yBuffer[i] = self.yBuffer[i-1]

        return xFiltered, yFiltered
