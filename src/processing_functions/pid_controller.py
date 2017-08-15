#!/usr/bin/env python

import rospy
import numpy as np
import cv2

class PIDController(object):

    def __init__(self,Kp=0.2, Ki=0.0, Kd=0.0, moveTime = 0.0, waitTime = 0.00):
        
        self.xDerivator = 0.0
        self.yDerivator = 0.0
        self.xIntegrator = 0.0
        self.yIntegrator = 0.0
        self.xIntegral = 0.0
        self.yIntegral = 0.0

        self.xError = 0.0
        self.yError = 0.0

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.dt = moveTime + waitTime
        self.oldTime = rospy.Time.now()

        self.InitializeFilter()

    def SetPIDTerms(self):

        if self.xError != None and self.yError != None:
            #Calculation for the P_term
            #Linear relationship between Error and output
            
            self.x_pTerm = self.Kp * self.xError
            self.y_pTerm = self.Kp * self.yError

            #Calculation for the I_term
            #Gets the accumulation of error over time to assist the P_term in pushing the drone

            self.xIntegrator = self.xFiltered * self.dt.to_sec()
            self.yIntegrator = self.yFiltered * self.dt.to_sec()

            self.xIntegral += self.xIntegrator
            self.yIntegral += self.yIntegrator

            if self.cx > self.xLower and self.cx < self.xUpper:
                self.xIntegral = 0.0
            if self.cy > self.yLower and self.cy < self.yUpper:
                self.yIntegral = 0.0

            self.x_iTerm = self.Ki * self.xIntegral
            self.y_iTerm = self.Ki * self.yIntegral

            #Calculation for the D_term
            #Computes the rate of change of our Error to compensate for overshooting the command
            self.xTemp = self.xD - self.xDerivator
            self.yTemp = self.yD - self.yDerivator
            
            if self.dt.to_sec() > 0.0:
                self.xDerivative = self.xTemp/self.dt.to_sec()
                self.yDerivative = self.yTemp/self.dt.to_sec()
            else:
                self.xDerivative = 0.0
                self.yDerivative = 0.0

            self.x_dTerm = self.Kd * self.xDerivative
            self.y_dTerm = self.Kd * self.yDerivative

            self.xDerivator = self.xD
            self.yDerivator = self.yD

        else:
            self.xIntegral = 0.0
            self.yIntegral = 0.0

        #return self.x_pTerm, self.y_pTerm, self.x_iTerm, self.y_iTerm, self.x_dTerm, self.y_dTerm


    #Sum up the three P,I,D terms to get the output Command
    #Return xspeed for roll and yspeed for pitch
    def GetPIDValues(self):

        if self.xError != None and self.yError != None:
            if self.cx < self.xLower or self.cx > self.xUpper:
                rospy.logwarn( "p:" + str(self.x_pTerm)+ " i:"+ str(self.x_iTerm)+" d:"+str(self.x_dTerm))
                xPID = (self.x_pTerm + self.x_iTerm + self.x_dTerm)/self.centerx
            else:
                xPID = 0.0

            if self.cy < self.yLower or self.cy > self.yUpper:
                yPID = (self.y_pTerm + self.y_iTerm + self.y_dTerm)/self.centery
            else:
                yPID = 0.0

        else:
            xPID = 0.0
            yPID = 0.0
            self.xIntegral = 0.0
            self.yIntegral = 0.0
        
        return xPID,yPID

        
    #Compute the desired SetPoint for the Drone - the center of the image
    #Set the desired window size for drone to hover in
    def SetPoint(self,image,windowSize=25):
        
        self.numRows, self.numCols, self.channels = image.shape
        self.centerx = self.numCols/2.0
        self.centery = self.numRows/2.0

        self.xLower = self.centerx-windowSize
        self.yLower = self.centery-windowSize
        self.xUpper = self.centerx+windowSize
        self.yUpper = self.centery+windowSize

        cv2.rectangle(image, (int(self.xLower), int(self.yLower)), (int(self.xUpper), int(self.yUpper)), (255,0,0), 2)


    #Calculate Error as a function of object's distance from desired setpoint (the center)
    #Perform a LowPass Filter on the Error Values for the Integral and Derivative Terms
    def UpdateError(self,cx,cy):
        
        self.cx = cx
        self.cy = cy

        if cx != None and cy != None:
            self.xError = self.centerx - self.cx
            self.yError = self.centery - self.cy
            self.xFiltered, self.yFiltered = self.LowPassFilter(self.xError, self.yError)
            self.xD, self.yD = self.LowPassFilter(-self.cx, -self.cy)
        
        else:
            self.xError = None
            self.yError = None
        
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
        
        self.filterSize = self.coef.size

        self.xBuffer = np.zeros(self.filterSize)
        self.yBuffer = np.zeros(self.filterSize)

    #Perform a LowPassFilter on Error values to remove noise and high-freq values
    #return filtered error values
    def LowPassFilter(self,xError,yError):
    
        xFiltered = 0.0
        yFiltered = 0.0
        
        self.xBuffer[0] = xError
        self.yBuffer[0] = yError

        for i in np.arange(self.filterSize):
            xFiltered += self.xBuffer[i] * self.coef[i]
            yFiltered += self.yBuffer[i] * self.coef[i]

        for i in np.arange(self.filterSize-1,0,-1):
            self.xBuffer[i] = self.xBuffer[i-1]
            self.yBuffer[i] = self.yBuffer[i-1]

        return xFiltered, yFiltered
    
    def ResetPID(self, P=None, I=None, D=None):
        
        if P != None:
            self.Kp = P
        if I != None:
            self.Ki = I
        if D != None:
            self.Kd = D
        
        self.InitializeFilter()
        self.oldTime = rospy.Time.now()

    def DrawArrow(self,image,xspeed,yspeed):
        dx=int((-100*xspeed)+self.centerx)
        dy=int((-100*yspeed)+self.centery)
        cv2.arrowedLine(image,(dx,dy),(int(self.centerx),int(self.centery)),(255,0,0),3)



