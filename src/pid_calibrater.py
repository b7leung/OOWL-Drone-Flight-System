#!/usr/bin/env python
#-*- coding utf-8 -*-
import pygame
import rospy
from drone_controller import BasicDroneController
from os.path import expanduser
#from processing_functions/pid_controller import PIDController

#from os import *
# global pygame color constants, in form (r,g,b) where 0 <= r/g/b <= 255
GREY = (192,192,192)

class Calibrater(object):



    def __init__(self):
        
        # initalize gui window (pygame)
        pygame.init()
        self.screen = pygame.display.set_mode((353, 576))
        pygame.display.set_caption("Calibrater Controller")
        (self.screen).fill(GREY)
	background = pygame.image.load(expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/PID_Calibrater.png")
	self.screen.blit(background,[0,0])
        pygame.display.update()
        # setup controller + its variables
        self.controller = BasicDroneController("Keyboard")
        self.editValue = None
        # config file path
        self.settingsPath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/calibrater_settings.txt"
        
        #self.pid = PIDController()
       # tempP=P
        #if press up:
        #tempP+1
        
        self.CALIBRATE_P = "p"
        self.CALIBRATE_I = "i"
        self.CALIBRATE_D = "d"
        P,I,D = self.GetSettings()
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.increment = 0.001
        
    def GetSettings(self):
    
        # read a text file as a list of lines
        # find the last line, change to a file you have
        fileHandle = open ( self.settingsPath,'r' )
        last = fileHandle.readlines()
        fileHandle.close()        
        
        last=str(last[len(last)-1]).split()
        rospy.logwarn(str(last))
        p, i, d = [float(x) for x in (last)]

        return p, i ,d
        
    def WriteAll(self):
        calibraterFile  = open(self.settingsPath, 'a') 
        calibraterFile.write(str(self.Kp)+" ")
        calibraterFile.write(str(self.Ki)+" ")
        calibraterFile.write(str(self.Kd)+"\n")
        calibraterFile.close()

    def setPID(self, Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
    def isZero(self, alpha):
        if(alpha < 0.0000000000000001):
            return 0.0
        else: return alpha
        
    def startController(self):
        # while gui is still running, continusly polls for keypresses
        gameExit = False
        while not gameExit:
            for event in pygame.event.get():
                # checking when keys are pressing down
                if event.type == pygame.KEYDOWN and self.controller is not None:

                    P,I,D = self.GetSettings()
                    self.setPID(P,I,D)
                    
                    if event.key == pygame.K_SPACE:
                        self.controller.SendLand()
                        print "Land"
                    elif event.key == pygame.K_ESCAPE:
                        self.controller.SendEmergency()
                        print "Emergency Land"
                    elif event.key == pygame.K_c:
                        self.controller.ToggleCamera()
                        print "toggle camera"
                    
                    else:
                    
                        if event.key == pygame.K_p:
                            if(self.editValue != self.CALIBRATE_P):
                                self.editValue = self.CALIBRATE_P
                            else: self.editValue = None
                        elif event.key == pygame.K_i:
                            if(self.editValue != self.CALIBRATE_I):
                                self.editValue = self.CALIBRATE_I
                            else: self.editValue = None
                        elif event.key == pygame.K_d:
                            if(self.editValue != self.CALIBRATE_D):
                                self.editValue = self.CALIBRATE_D
                            else: self.editValue = None
                        elif event.key == pygame.K_UP:
                            if(self.editValue ==self.CALIBRATE_P):
                                self.Kp += self.increment
                                self.Kp = self.isZero(self.Kp)
                                rospy.logwarn("P: " + str(self.Kp))
                                self.WriteAll()
                            elif(self.editValue == self.CALIBRATE_I):
                                self.Ki += self.increment
                                self.Ki = self.isZero(self.Ki)

                                rospy.logwarn("I: " + str(self.Ki))
                                self.WriteAll()
                            elif(self.editValue == self.CALIBRATE_D):
                                self.Kd += self.increment
                                self.Kd = self.isZero(self.Kd)

                                rospy.logwarn("D: " + str(self.Kd))
                                self.WriteAll()

                        elif event.key == pygame.K_DOWN:
                            if(self.editValue == self.CALIBRATE_P):
                                self.Kp = self.Kp - self.increment
                                self.Kp = self.isZero(self.Kp)

                                rospy.logwarn("P: " + str(self.Kp))
                                self.WriteAll()

                            elif(self.editValue == self.CALIBRATE_I):
                                self.Ki = self.Ki - self.increment
                                self.Ki = self.isZero(self.Ki)

                                rospy.logwarn("I: " + str(self.Ki))
                                self.WriteAll()

                            elif(self.editValue == self.CALIBRATE_D):
                                self.Kd = self.Kd - self.increment
                                self.Kd = self.isZero(self.Kd)

                                rospy.logwarn("D: " + str(self.Kd))
                                self.WriteAll()


                        

                if event.type == pygame.KEYUP:
                    if (event.key == pygame.K_w or event.key == pygame.K_s or event.key == pygame.K_a or event.key == pygame.K_d
                    or event.key == pygame.K_q or event.key == pygame.K_e or event.key == pygame.K_r or event.key == pygame.K_f):
                        pass
                if event.type == pygame.QUIT:
                    gameExit = True

        pygame.display.quit()
        pygame.quit()

if __name__=="__main__":
    rospy.init_node('KeyboardController')
    myController = Calibrater()
    myController.startController()
