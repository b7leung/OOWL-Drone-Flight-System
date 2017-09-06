#!/usr/bin/env python

import pygame
import rospy
from drone_controller import BasicDroneController
from os.path import expanduser

GREY = (192,192,192)
CALIBRATE_P = "p"
CALIBRATE_I = "i"
CALIBRATE_D = "d"

class Calibrater(object):


    def __init__(self):
        
        # initalizing gui window (pygame)
        pygame.init()
        self.screen = pygame.display.set_mode((353, 576))
        pygame.display.set_caption("Calibrater Controller")
        (self.screen).fill(GREY)
	background = pygame.image.load(expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/PID_Calibrater.png")
	self.screen.blit(background,[0,0])
        pygame.display.update()

        self.controller = BasicDroneController("Keyboard")
        self.editValue = None

        # config file path
        self.settingsPath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/calibrater_settings.txt"
        
        self.Kp, self.Ki, self.Kd = self.GetSettings()

        self.increment = 0.001
        

    # reads the PID values from the last line of the file
    def GetSettings(self):
    
        # read a text file as a list of lines
        # find the last line, change to a file you have
        fileHandle = open ( self.settingsPath,'r' )
        last = fileHandle.readlines()
        fileHandle.close()        
        
        last=str(last[len(last)-1]).split()
        p, i, d = [float(x) for x in last]

        return p, i ,d
    

    # writes the current PID values to the file
    def WriteAll(self):

        calibraterFile  = open(self.settingsPath, 'a') 
        calibraterFile.write(str(self.Kp)+" ")
        calibraterFile.write(str(self.Ki)+" ")
        calibraterFile.write(str(self.Kd)+"\n")
        calibraterFile.close()
        rospy.logwarn( "P = " + str(self.Kp) + "  I = " + str(self.Ki) + "  D = " + str(self.Kd) )


    def setPID(self, Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        

    def isZero(self, alpha):

        if(alpha < 0.0000000000000001):
            return 0.0
        else:
            return alpha
        

    def startController(self):

        # while gui is still running, continusly polls for keypresses
        gameExit = False
        while not gameExit:
            for event in pygame.event.get():
                # checking when keys are pressing down
                if event.type == pygame.KEYDOWN and self.controller is not None:

                    #P,I,D = self.GetSettings()
                    #self.setPID(P,I,D)
                    
                    if event.key == pygame.K_SPACE:
                        self.controller.SendLand()
                        print "Land"

                    elif event.key == pygame.K_ESCAPE:
                        self.controller.SendEmergency()
                        print "Emergency Land"

                    elif event.key == pygame.K_c:
                        self.controller.ToggleCamera()
                        print "toggle camera"
                    
                    elif event.key == pygame.K_p:
                        if(self.editValue != CALIBRATE_P):
                            self.editValue = CALIBRATE_P
                        else: self.editValue = None

                    elif event.key == pygame.K_i:
                        if(self.editValue != CALIBRATE_I):
                            self.editValue = CALIBRATE_I
                        else: self.editValue = None

                    elif event.key == pygame.K_d:
                        if(self.editValue != CALIBRATE_D):
                            self.editValue = CALIBRATE_D
                        else: self.editValue = None

                    elif event.key == pygame.K_UP:

                        if(self.editValue == CALIBRATE_P):
                            self.Kp += self.increment
                            self.Kp = self.isZero(self.Kp)
                            #rospy.logwarn("P: " + str(self.Kp))
                            self.WriteAll()
                        elif(self.editValue == CALIBRATE_I):
                            self.Ki += self.increment
                            self.Ki = self.isZero(self.Ki)

                            #rospy.logwarn("I: " + str(self.Ki))
                            self.WriteAll()
                        elif(self.editValue == CALIBRATE_D):
                            self.Kd += self.increment
                            self.Kd = self.isZero(self.Kd)

                            #rospy.logwarn("D: " + str(self.Kd))
                            self.WriteAll()

                    elif event.key == pygame.K_DOWN:

                        if(self.editValue == CALIBRATE_P):
                            self.Kp = self.Kp - self.increment
                            self.Kp = self.isZero(self.Kp)

                            #rospy.logwarn("P: " + str(self.Kp))
                            self.WriteAll()

                        elif(self.editValue == CALIBRATE_I):
                            self.Ki = self.Ki - self.increment
                            self.Ki = self.isZero(self.Ki)

                            #rospy.logwarn("I: " + str(self.Ki))
                            self.WriteAll()

                        elif(self.editValue == CALIBRATE_D):
                            self.Kd = self.Kd - self.increment
                            self.Kd = self.isZero(self.Kd)

                            #rospy.logwarn("D: " + str(self.Kd))
                            self.WriteAll()

                    elif event.key == pygame.K_KP4:
                        self.Kp = self.isZero( self.Kp + self.increment )
                        self.WriteAll()
                    elif event.key == pygame.K_KP1:
                        self.Kp = self.isZero( self.Kp - self.increment )
                        self.WriteAll()

                    elif event.key == pygame.K_KP5:
                        self.Ki = self.isZero( self.Ki + self.increment )
                        self.WriteAll()
                    elif event.key == pygame.K_KP2:
                        self.Ki = self.isZero( self.Ki - self.increment )
                        self.WriteAll()

                    elif event.key == pygame.K_KP6:
                        self.Kd = self.isZero( self.Kd + self.increment )
                        self.WriteAll()
                    elif event.key == pygame.K_KP3:
                        self.Kd = self.isZero( self.Kd- self.increment )
                        self.WriteAll()


                if event.type == pygame.KEYUP:
                    if (event.key == pygame.K_w or event.key == pygame.K_s or event.key == pygame.K_a or event.key == pygame.K_d
                    or event.key == pygame.K_q or event.key == pygame.K_e or event.key == pygame.K_r or event.key == pygame.K_f):
                        pass
                if event.type == pygame.QUIT:
                    gameExit = True

        pygame.display.quit()
        pygame.quit()


    # this is called by ROS when the node shuts down
    def ShutdownTasks(self):
        
        # at maximum, only keeps 20 pid values in the file
        lines = open(self.settingsPath).readlines()
        numLines=20
        if len(lines) < 20:
            numLines = len(lines)
        
        open(self.settingsPath, 'w').writelines(lines[-numLines:len(lines)])
        

if __name__=="__main__":

    rospy.init_node('KeyboardController')
    myController = Calibrater()
    rospy.on_shutdown(myController.ShutdownTasks)
    myController.startController()
