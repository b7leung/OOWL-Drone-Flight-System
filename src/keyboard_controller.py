#!/usr/bin/env python

import pygame
import rospy
from drone_controller import BasicDroneController
from os.path import expanduser

# global pygame color constants, in form (r,g,b) where 0 <= r/g/b <= 255
GREY = (192,192,192)

class KeyboardController(object):


    def __init__(self):
        
        # initalize gui window (pygame)
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Keyboard Controller")
        (self.screen).fill(GREY)
        background = pygame.image.load(expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/KeyboardCommands4.png")
        self.screen.blit(background,[0,0])
        pygame.display.update()

        # setup controller + its variables
        self.controller = BasicDroneController("Keyboard")
        self.speed = 1
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0



    def startController(self):
        # while gui is still running, continusly polls for keypresses
        gameExit = False
        while not gameExit:
            for event in pygame.event.get():
                # checking when keys are pressing down
                if event.type == pygame.KEYDOWN and self.controller is not None:

                    if event.key == pygame.K_t:
                        #switches camera to bottom when it launches
                        self.controller.SendTakeoff()
                        self.controller.SwitchCamera(1)
                        print( "Takeoff")
                    elif event.key == pygame.K_g:
                        self.controller.SendLand()
                        rospy.logwarn("-------- LANDING DRONE --------") 
                        print ("Land")
                    elif event.key == pygame.K_ESCAPE:
                        self.controller.SendEmergency()
                        print ("Emergency Land")
                    elif event.key == pygame.K_c:
                        self.controller.ToggleCamera()
                        print ("toggle camera")
                    elif event.key == pygame.K_z:
                        self.controller.FlatTrim()
                    else:
                    
                        if event.key == pygame.K_w:
                            self.pitch = self.speed
                        elif event.key == pygame.K_s:
                            self.pitch = self.speed*-1
                        elif event.key == pygame.K_a:
                            self.roll = self.speed
                            print ("Roll Left")
                        elif event.key == pygame.K_d:
                            self.roll = self.speed*-1
                            print ("Roll Right")
                        elif event.key == pygame.K_q:
                            self.yaw_velocity = self.speed
                            print ("Yaw Left")
                        elif event.key == pygame.K_e:
                            self.yaw_velocity = self.speed*-1
                            print ("Yaw Right")
                        elif event.key == pygame.K_r:
                            self.z_velocity = self.speed*1
                            print ("Increase Altitude")
                        elif event.key == pygame.K_f:
                            self.z_velocity = self.speed*-1
                            print ("Decrease Altitude")
                            

                        self.controller.SetCommand(self.roll, self.pitch,
                        self.yaw_velocity, self.z_velocity)

                if event.type == pygame.KEYUP:
                    if (event.key == pygame.K_w or event.key == pygame.K_s or event.key == pygame.K_a or event.key == pygame.K_d
                    or event.key == pygame.K_q or event.key == pygame.K_e or event.key == pygame.K_r or event.key == pygame.K_f):
                        self.pitch = 0
                        self.roll = 0
                        self.z_velocity = 0
                        self.yaw_velocity = 0
                        self.controller.SetCommand(self.roll, self.pitch,
                        self.yaw_velocity, self.z_velocity)

                if event.type == pygame.QUIT:
                    gameExit = True

        pygame.display.quit()
        pygame.quit()

if __name__=="__main__":
    rospy.init_node('KeyboardController')
    myController = KeyboardController()
    myController.startController()
