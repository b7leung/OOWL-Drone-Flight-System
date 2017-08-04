#!/usr/bin/env python

import rospy
import numpy as np

 class PIDController(object):

    def __init__(self,Kp=0.5, Ki=0.0, Kd=0.5, integralMax=1, integralMin=-1, moveTime = 0.1, waitTime = 0.04):
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.xDerivator = 0.0
        self.yDerivator = 0.0
        self.xIntegrator = 0.0
        self.yIntegrator = 0.0

        self.xError = 0.0
        self.yError = 0.0

        self.dt = moveTime + waitTime
        self.oldTime = rospy.Time.now()

    def pTerm(self):

        self.x_pTerm = self.Kp * self.xError
        self.y_pTerm = self.Kp * self.yError

    def iTerm(self):
    
        self.x_iTerm = self.Ki * self.xIntegral
        self.y_iTerm = self.Ki * self.yIntegral
        self.UpdateITerm()

    def dTerm(self):

        self.xResult, self.yResult = self.LowPassFilter(self.xError, self.yError)
        self.xTemp = self.xFiltered - self.xDerivator
        self.yTemp = self.yFiltered - self.yDerivator
        
        if self.dt > 0.0:
            self.xDerivative = self.xTemp/self.dt
            self.yDerivative = self.yTemp/self.dt
        else:
            self.xDerivative = 0.0
            self.yDerivative = 0.0

        self.x_dTerm = self.Kd * self.xDerivative
        self.y_dTerm = self.Kd * self.yDerivative

    def GetPIDValues(self):

        xPID = self.x_pTerm + self.x_iTerm + self.x_dTerm
        yPID = self.y_pTerm + self.y_iTerm + self.y_dTerm
        
        return xPID,yPID

    def SetPoint(self,image):
        
        self.numRows, self.numCols, self.channels = image.shape
        self.centerx = self.numCols/2
        self.centery = self.numRows/2

    def UpdateError(self,cx,cy):
        
        self.xError = (self.centerx - cx)/float(self.centerx)
        self.yError = (self.centery - cy)/float(self.centery)

    def SetPID(self,Kp,Ki,Kd):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def UpdateDTerm(self):
        
        self.xDerivator = self.xFiltered
        self.yDerivator = self.yFiltered

    def UpdateITerm(self):

        self.xIntegrator = self.xError * dt
        self.yIntegrator = self.yError * dt
        
        self.xIntegral += self.xIntegrator
        self.yIntegral += self.yIntegrator

    def UpdateTimeDelta(self):
        
        self.timeNow = rospy.Time.now()
        self.dt = self.timeNow - self.oldTime

    def UpdateTime(self):

        self.oldTime = rospy.Time.now()

    def LowPassFilter(self,xError, yError)

        coef = np.array([0.00498902, 0.00567655, 0.00768429, 0.01092849, 0.01526534,
                0.02049766, 0.02638413, 0.03265082, 0.03900430, 0.04514569,
                0.05078516, 0.05565588, 0.05952694, 0.06221459, 0.06359114,
                0.06359114, 0.06221459, 0.05952694, 0.05565588, 0.05078516,
                0.04514569, 0.03900430, 0.03265082, 0.02638413, 0.02049766,
                0.01526534, 0.01092849, 0.00768429, 0.00567655, 0.00498902])
        
        filterSize = coef.size

        xBuffer = np.zeros(filterSize)
        yBuffer = np.zeros(filterSize)

        xBuffer[0] = xError
        yBuffer[0] = yError

        for i in np.arange(filterSize):
            xFiltered += xBuffer[i] * coef[i]
            yFiltered += yBuffer[i] * coef[i]

        for i in np.arange(filterSize-1,0,-1):
            xBuffer[i] = xBuffer[i-1]
            yBuffer[i] = yBuffer[i-1]

        return xFiltered, yFiltered
