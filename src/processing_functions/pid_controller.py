#!/usr/bin/env python

import rospy

 class PIDController(object):

    def __init__(self,Kp=0.5, Ki=0.0, Kd=0.5, Derivator=0, Integrator=0, integralMax=1, integralMin=-1):
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.xDerivator = Derivator
        self.yDerivator = Derivator
        self.xIntegrator = Integarator
        self.yIntegrator = Integrator

        self.xError = 0.0
        self.yError = 0.0

    def pTerm(self):

        self.x_pTerm = self.Kp * self.xError
        self.y_pTerm = self.Kp * self.yError

    def iTerm(self):
    
        self.x_iTerm = self.Ki * self.xIntegral
        self.y_iTerm = self.Ki * self.yIntegral
        self.UpdateITerm()

    def dTerm(self):

        self.xTemp = self.xError - self.xDerivator
        self.yTemp = self.yError - self.yDerivator
        
        self.xDerivative = self.xTemp/self.dt
        self.yDerivative = self.yTemp/self.dt

        self.UpdateDTerm()
        self.UpdateTime()

        self.x_dTerm = self.Kd * self.xDerivative
        self.y_dTerm = self.Kd * self.yDerivative

    def SetPIDController(self):

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
        
        self.xDerivator = self.xError
        self.yDerivator = self.yError

    def UpdateITerm(self):

        self.xIntegrator = self.xError * dt
        self.yIntegrator = self.yError * dt
        
        self.xIntegral += self.xIntegrator
        self.yIntegral += self.yIntegrator

    def GetTime(self):
        
        self.timeNow = rospy.Time.now()

    def Updatetime(self):

        self.dt = self.timeNow - self.oldTime
        self.oldTime = self.timeNow
