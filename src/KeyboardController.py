#!/usr/bin/env python

import pygame
import rospy
from drone_controller import BasicDroneController

# global pygame color constants, in form (r,g,b) where 0 <= r/g/b <= 255
WHITE = (255, 255, 255)
BLACK = (0,0,0)
DARK_BLUE= (0,0,128)
RED = (255, 0, 0)
GREY = (192,192,192)

BUTTON_HEIGHT = 65
BUTTON_LINE_THICKNESS =0
BUTTON_COLOR = BLACK
BUTTON_SPACING = 15
Q_BUTTON_X = 30
Q_BUTTON_Y = 30
BUTTON_FONT = None
LETTER_X_OFFSET = 7
LETTER_Y_OFFSET = 5

class KeyboardController(object):


    def __init__(self):
        
        # initalize gui window (pygame)
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Keyboard Controller")
        (self.screen).fill(GREY)
        global BUTTON_FONT
        BUTTON_FONT = pygame.font.SysFont(None, 90)

        # setup controller + its variables
        self.controller = BasicDroneController()
        self.speed = 1
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0

        # show button graphics on gui
        """self.q_key_graphic = pygame.draw.rect(self.screen, BUTTON_COLOR,
            (Q_BUTTON_X,Q_BUTTON_Y,BUTTON_HEIGHT,BUTTON_HEIGHT),BUTTON_LINE_THICKNESS)
        q_text = BUTTON_FONT.render("Q", True, WHITE)
        self.screen.blit(q_text,(self.q_key_graphic.x+LETTER_X_OFFSET,self.q_key_graphic.y+LETTER_Y_OFFSET))

        self.w_key_graphic = pygame.draw.rect(self.screen, BUTTON_COLOR,
            (self.q_key_graphic.x+BUTTON_HEIGHT+BUTTON_SPACING,Q_BUTTON_Y,BUTTON_HEIGHT,BUTTON_HEIGHT),BUTTON_LINE_THICKNESS)
        w_text = BUTTON_FONT.render("W", True, WHITE)
        self.screen.blit(w_text,(self.w_key_graphic.x+LETTER_X_OFFSET,self.w_key_graphic.y+LETTER_Y_OFFSET))

        self.e_key_graphic = pygame.draw.rect(self.screen, BUTTON_COLOR,
            (self.w_key_graphic.x+BUTTON_HEIGHT+BUTTON_SPACING,Q_BUTTON_Y,BUTTON_HEIGHT,BUTTON_HEIGHT),BUTTON_LINE_THICKNESS)
        e_text = BUTTON_FONT.render("E", True, WHITE)
        self.screen.blit(w_text,(self.w_key_graphic.x+LETTER_X_OFFSET,self.w_key_graphic.y+LETTER_Y_OFFSET))
        """


        pygame.display.update()
        
    def startController(self):
        # while gui is still running, continusly polls for keypresses
        gameExit = False
        while not gameExit:
            for event in pygame.event.get():
                # checking when keys are pressing down
                if event.type == pygame.KEYDOWN and self.controller is not None:

                    if event.key == pygame.K_t:
                        self.controller.SendTakeoff()
                        print "Takeoff"
                    elif event.key == pygame.K_SPACE:
                        self.controller.SendLand()
                        print "Land"
                    elif event.key == pygame.K_ESCAPE:
                        self.controller.SendEmergency()
                        print "Emergency Land"
                    else:
                    
                        if event.key == pygame.K_w:
                            self.pitch = self.speed
                            print "Pitch Forwards"
                        elif event.key == pygame.K_s:
                            self.pitch = self.speed*-1
                            print "Pitch Backwards"
                        elif event.key == pygame.K_a:
                            self.roll = self.speed
                            print "Roll Left"
                        elif event.key == pygame.K_d:
                            self.roll = self.speed*-1
                            print "Roll Right"
                        elif event.key == pygame.K_q:
                            self.yaw_velocity = self.speed
                            print "Yaw Left"
                        elif event.key == pygame.K_e:
                            self.yaw_velocity = self.speed*-1
                            print "Yaw Right"
                        elif event.key == pygame.K_r:
                            self.z_velocity = self.speed*1
                            print "Increase Altitude"
                        elif event.key == pygame.K_f:
                            self.z_velocity = self.speed*-1
                            print "Decrease Altitude"

                        self.controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

                if event.type == pygame.KEYUP:
                    self.pitch = 0
                    self.roll = 0
                    self.z_velocity = 0
                    self.yaw_velocity = 0
                    self.controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

                if event.type == pygame.QUIT:
                    gameExit = True

        pygame.display.quit()
        pygame.quit()

if __name__=="__main__":
    rospy.init_node('KeyboardController')
    myController = KeyboardController()
    myController.startController()
