#!/usr/bin/env python

import pygame
import rospy
#from drone_controller import BasicDroneController

class KeyboardController(object):

	def __init__(self):
		pygame.init()
		#pygame.key.set_repeat(100,100)
		screen = pygame.display.set_mode((640, 480))
		pygame.display.set_caption("Keyboard Controller")

		#self.controller = BasicDroneController()
		self.speed = 1
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0
		self.z_velocity = 0
		
	def startController(self):
		gameExit = False
		while not gameExit:
			for event in pygame.event.get():
				if event.type == pygame.KEYDOWN:
					if event.key == pygame.K_w:
						self.pitch = 1
						print "Pitch Forwards"
					if event.key == pygame.K_s:
						self.pitch = -1
						print "Pitch Backwards"
					if event.key == pygame.K_a:
						print "Roll Left"
					if event.key == pygame.K_d:
						print "Roll Right"
					if event.key == pygame.K_q:
						print "Yaw Left"
					if event.key == pygame.K_e:
						print "Yaw Right"
					if event.key == pygame.K_r:
						print "Increase Altitude"
					if event.key == pygame.K_f:
						print "Decrease Altitude"
					if event.key == pygame.K_t:
						print "Takeoff"
					if event.key == pygame.K_SPACE:
						print "Land"
					if event.key == pygame.K_ESCAPE:
						print "Emergency Land"
					print "PITCH:", self.pitch
					#controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

				if event.type == pygame.KEYUP:
					print "stop"
					self.pitch = 0
					print "PITCH:", self.pitch
					#controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
				if event.type == pygame.QUIT:
					gameExit = True
		pygame.display.quit()
		pygame.quit()

if __name__=="__main__":
	rospy.init_node('KeyboardController')
	controller = KeyboardController()
	controller.startController()
