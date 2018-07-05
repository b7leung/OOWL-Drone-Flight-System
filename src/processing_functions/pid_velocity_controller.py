#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from std_msgs.msg import Int32, Float32


class PIDController(object):

    def __init__(self, Kp=0.138, Ki=0.0018, Kd=0.048):
        
        # units = M
        self.f = 715.6186 
        self.b = 5.1371 
        
        # Setting the desired window size for drone to hover in, relative to the center of the image
        windowSize = 1

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.xDerivator = 0.0
        self.yDerivator = 0.0
        self.xIntegral = 0.0
        self.yIntegral = 0.0

        self.xP_error = 0.0
        self.yP_error = 0.0
        self.xI_error = 0.0
        self.yI_error = 0.0
        self.xD_error = 0.0
        self.yD_error = 0.0

        #self.dt = moveTime + waitTime
        self.oldTime = None

        self.InitializeGaussianFilter()
        self.InitializeMedianFilter()
        self.pub_pid_xpcomp= rospy.Publisher('pid_xpcomp', Float32, queue_size = 10)
        self.pub_pid_xicomp = rospy.Publisher('pid_xicomp', Float32, queue_size = 10)
        self.pub_pid_xdcomp = rospy.Publisher('pid_xdcomp', Float32, queue_size = 10)
        self.pub_pid_ypcomp= rospy.Publisher('pid_ypcomp', Float32, queue_size = 10)
        self.pub_pid_yicomp = rospy.Publisher('pid_yicomp', Float32, queue_size = 10)
        self.pub_pid_ydcomp = rospy.Publisher('pid_ydcomp', Float32, queue_size = 10)

    # given error, gives output to reach setpoint
    def getOutput(self, vx, vy, altitude):

        if vx == None or vy == None:
            return 0, 0

        else:
            dt = self.getDt()

            # getting P error
            xP_error = self.Odom_Vx - vx
            yP_error = self.Odom_Vy - vy
            x_pTerm = self.Kp * xP_error
            y_pTerm = self.Kp * yP_error
            self.pub_pid_xpcomp.publish(x_pTerm)
            self.pub_pid_ypcomp.publish(y_pTerm)
            
            #getting I error
            xI_error, yI_error = xP_error, yP_error

            self.xIntegral += xI_error * dt
            self.yIntegral += yI_error * dt

            x_iTerm = self.Ki * self.xIntegral
            y_iTerm = self.Ki * self.yIntegral
            self.pub_pid_xicomp.publish(x_iTerm)
            self.pub_pid_yicomp.publish(y_iTerm)

            #getting D error
            xD_error, yD_error = xP_error, yP_error
            if dt!=0:
                x_dTerm = self.Kd*((xP_error - self.xDerivator)/dt)
                y_dTerm = self.Kd*((yP_error - self.yDerivator)/dt)
            else:
                x_dTerm = 0
                y_dTerm = 0
            self.pub_pid_xdcomp.publish(x_dTerm)
            self.pub_pid_ydcomp.publish(y_dTerm)
            self.xDerivator = xP_error
            self.yDerivator = yP_error

            x_out = x_pTerm + x_iTerm + x_dTerm
            y_out = y_pTerm + y_iTerm + y_dTerm
            rospy.logwarn("X_P = " + str(x_pTerm) +" X_I= " + str(x_iTerm)+ " X_D= " + str(x_dTerm))
            return x_out, y_out


    def getDt(self):
        
        timeNow = rospy.Time.now()

        if self.oldTime == None:
            self.oldTime = timeNow
            return 0

        dt = timeNow - self.oldTime
        self.oldTime = timeNow
        return dt.to_sec()
    

    # Calculate Error as a function of object's distance from desired setpoint (the center)
    # Perform a Gaussian Filter on the Error Values for the Integral and Derivative Terms
    def UpdateError(self,cx,cy,altitude):
        
        self.cx = cx
        self.cy = cy

        if cx != None and cy != None:

            # for P
            self.xP_error = (self.centerx - self.cx) * (altitude - self.b) / self.f
            self.yP_error = (self.centery - self.cy) * (altitude - self.b) / self.f
            
            # for I
            self.xI_error, self.yI_error = self.GaussianFilter(self.xP_error, self.yP_error)
            

            # for D
            xTemp = -self.cx * (altitude - self.b) / self.f
            yTemp = -self.cy * (altitude - self.b) / self.f


            self.xD_error, self.yD_error= self.GaussianFilter(xTemp, yTemp)
        
        else:
            self.xP_error = None
            self.yP_error = None


    def SetPIDTerms(self):

        if self.xP_error != None and self.yP_error != None:

            # Calculation for the P term:
            # Linear relationship between Error and output
            
            self.x_pTerm = self.Kp * self.xP_error
            self.y_pTerm = self.Kp * self.yP_error

            # Calculation for the I_term;
            # Gets the accumulation of error over time to assist the P_term in pushing the drone

            self.xIntegral += self.xI_error * self.dt.to_sec()
            self.yIntegral += self.yI_error * self.dt.to_sec()

            '''if self.cx > self.xLower and self.cx < self.xUpper:
                self.xIntegral = 0.0
            if self.cy > self.yLower and self.cy < self.yUpper:
                self.yIntegral = 0.0'''

            self.x_iTerm = self.Ki * self.xIntegral
            self.y_iTerm = self.Ki * self.yIntegral

            # Calculation for the D term;
            # Computes the rate of change of our Error to compensate for overshooting the command
            
            if self.dt.to_sec() > 0.0:

                self.xDerivative = (self.xD_error - self.xDerivator) / self.dt.to_sec()
                self.yDerivative = (self.yD_error - self.yDerivator) / self.dt.to_sec()

            else:
                self.xDerivative = 0.0
                self.yDerivative = 0.0

            x_dTemp = self.Kd * self.xDerivative
            y_dTemp = self.Kd * self.yDerivative

            self.x_dTerm, self.y_dTerm = self.MedianFilter(x_dTemp, y_dTemp)

            self.xDerivator = self.xD_error
            self.yDerivator = self.yD_error

            # if the P and D term are the same sign, it means the drone is drifting in the opposite
            # direction as directed, therefore increase the proportional term to push it in the right direction.
            '''if self.x_pTerm > 0.0 and self.x_dTerm > 0.0:
                #self.x_pTerm = self.x_pTerm * 1.2
                self.x_pTerm = 0.0
                self.x_iTerm = 0.0
                self.x_dTerm = 0.0
            elif self.x_pTerm < 0.0 and self.x_dTerm < 0.0:
                #self.x_pTerm = self.x_pTerm * 1.2
                self.x_pTerm = 0.0
                self.x_iTerm = 0.0
                self.x_dTerm = 0.0
            if self.y_pTerm > 0.0 and self.y_dTerm > 0.0:
                #self.y_pTerm = self.y_pTerm * 1.2
                self.y_pTerm = 0.0
                self.y_iTerm = 0.0
                self.y_dTerm = 0.0
            elif self.y_pTerm < 0.0 and self.y_dTerm < 0.0:
                #self.y_pTerm = self.y_pTerm * 1.2
                self.y_pTerm = 0.0
                self.y_iTerm = 0.0
                self.y_dTerm = 0.0'''
        else:
            self.xIntegral = 0.0
            self.yIntegral = 0.0

        #return self.x_pTerm, self.y_pTerm, self.x_iTerm, self.y_iTerm, self.x_dTerm, self.y_dTerm


    # Sum up the three P,I,D terms to get the output Command
    # Return xspeed for roll and yspeed for pitch
    def GetPIDValues(self):

        if self.xP_error != None and self.yP_error != None:
      
            if self.cx < self.xLower or self.cx > self.xUpper:
               # rospy.logwarn( "p:" + str(self.x_pTerm)+ " i:"+ str(self.x_iTerm)+" d:"+str(self.x_dTerm))
                xPID = (self.x_pTerm + self.x_iTerm + self.x_dTerm)/self.centerx
            else:
                xPID = 0.0

            if self.cy < self.yLower or self.cy > self.yUpper:
                yPID = (self.y_pTerm + self.y_iTerm + self.y_dTerm)/self.centery
            else:
                yPID = 0.0

            '''rospy.logwarn(str(self.x_pTerm/self.centerx) + " + " + str(self.x_iTerm/self.centerx) + " + " + str(self.x_dTerm/self.centerx))

            rospy.logwarn(str(self.y_pTerm/self.centery) + " + " + str(self.y_iTerm/self.centery) + " + " + str(self.y_dTerm/self.centery))'''

            """
            rospy.logwarn("dt: "+ str(self.dt.to_sec()))
            rospy.logwarn("mag: "+ str(np.sqrt(xPID**2+yPID**2)))
            """
        else:
            xPID = 0.0
            yPID = 0.0
            self.xIntegral = 0.0
            self.yIntegral = 0.0

        return xPID,yPID 


    # Filter coefficients copied from github.com/raultrom/ardrone_velocity
    # https://github.com/raultron/ardrone_velocity/blob/master/filtervelocity.cpp
    def InitializeGaussianFilter(self):

        self.coef = np.array([0.00498902, 0.00567655, 0.00768429, 0.01092849, 0.01526534,
                0.02049766, 0.02638413, 0.03265082, 0.03900430, 0.04514569,
                0.05078516, 0.05565588, 0.05952694, 0.06221459, 0.06359114,
                0.06359114, 0.06221459, 0.05952694, 0.05565588, 0.05078516,
                0.04514569, 0.03900430, 0.03265082, 0.02638413, 0.02049766,
                0.01526534, 0.01092849, 0.00768429, 0.00567655, 0.00498902])
        
        self.filterSize = self.coef.size

        self.xBuffer = np.zeros(self.filterSize)
        self.yBuffer = np.zeros(self.filterSize)


    # Perform a Gaussian filter on Error values to remove noise and high-freq values
    # return filtered error values
    def GaussianFilter(self,xError,yError):
    
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
   

    def InitializeMedianFilter(self):

        self.bufferSize = 3
        self.xMedBuffer = np.zeros(self.bufferSize)
        self.yMedBuffer = np.zeros(self.bufferSize)


    def MedianFilter(self,xTerm,yTerm):

        self.xMedBuffer[0] = xTerm
        self.yMedBuffer[0] = yTerm

        xMedian = np.median(self.xMedBuffer)
        yMedian = np.median(self.yMedBuffer)

        for i in np.arange(self.bufferSize-1, 0, -1):
            self.xMedBuffer[i] = self.xMedBuffer[i-1]
            self.yMedBuffer[i] = self.yMedBuffer[i-1]

        return xMedian, yMedian


    def ResetPID(self, P=None, I=None, D=None):
        rospy.logwarn("***** Resetting PID Values *****")
        
        if P != None:
            self.Kp = P
        if I != None:
            self.Ki = I
        if D != None:
            self.Kd = D
        
        self.InitializeGaussianFilter()
        self.oldTime = None

        self.xDerivator = 0.0
        self.yDerivator = 0.0
        self.xIntegral = 0.0
        self.yIntegral = 0.0

        self.xP_error = 0.0
        self.yP_error = 0.0
        self.xI_error = 0.0
        self.yI_error = 0.0
        self.xD_error = 0.0
        self.yD_error = 0.0


    def DrawArrow(self,image,xspeed,yspeed):

        dx=int((-100*xspeed)+self.centerx)
        dy=int((-100*yspeed)+self.centery)
        cv2.arrowedLine(image,(dx,dy),(int(self.centerx),int(self.centery)),(255,0,0),3)


    def ReturnPIDvalues(self):

        return self.Kp, self.Ki, self.Kd

