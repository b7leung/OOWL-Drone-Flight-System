#!/usr/bin/env python

import sys

#import the ROS libraries and load the manifest file which through <depend packages=/// /> will give us access to the project dependences
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

#Import types of messeges we want
from sensor_msgs.msg import Image #for recieving video feed
from ardrone_autonomy.msg import Navdata

#Import service type for toggle cam
from std_srvs.srv import Empty

#load the controller that has the drone controlling functions
from drone_controller import BasicDroneController

#import opencv and numpy for image processing
import cv2
from numpy import *
import time
from cv_bridge import CvBridge, CvBridgeError

import argparse

class DroneVideo(object):

	def __init__(self):

		super(DroneVideo, self).__init__()		
		self.bridge=CvBridge()

		#create a pulisher for opencv image
#		self.image_pub = rospy.Publisher("image_topic_2", Image)

		#Subscribe to the drones video feed
		self.video=rospy.Subscriber('/ardrone/image_raw', Image, self.ROStoCVImage)

		#initialization for recording video
		fourcc=cv2.VideoWriter_fourcc(*'XVID')
		self.out=cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))		

	def ROStoCVImage(self,data):

		#convert ROS Image to OpenCV Image
		cv_image=self.bridge.imgmsg_to_cv2(data, "bgr8")
		#get size of image
		height, width, channels = cv_image.shape
		size = cv_image.size
		#Save the Video
		self.out.write(cv_image)
		#Display the Video		
		self.DetectColor(cv_image)
		cv2.waitKey(3)

	def DetectColor(self,image):
		
		# define the list of boundaries (order is BGR values)
		#starts with detecting red, blue yellow and then gray
		
		#bgr_boundaries = [
		#	([0,18,220],[150,150,255])
		#	]
		
		hsv_boundaries = [
			([0, 150, 200],[7, 255, 255])
			]

		hsv_boundaries2 = [([170, 140, 150],[179, 255, 255])
			]

		hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		#create numpy arrays from these colors
		lower= array(hsv_boundaries[0][0],dtype = "uint8")
		upper= array(hsv_boundaries[0][1],dtype = "uint8")
		lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
		upper2=array(hsv_boundaries2[0][1], dtype = "uint8")
		#find colors within the boundaries for each color
		mask1 = cv2.inRange(hsv_image,lower,upper)
		mask2 = cv2.inRange(hsv_image,lower2,upper2)
		mask = cv2.bitwise_or(mask1,mask2,mask=None)
		output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)

		numcols = len(mask)
		numrows = len(mask[0])
		centerx=numrows/2
		centery=numcols/2

		xlower=centerx-60 #left xvalue
		ylower=centery-60 #"top" yvalue
		xupper=centerx+60 #right xvalue
		yupper=centery+60 #"bottom" yvalue
		alphax=.03
		alphay=.02
	
		M=cv2.moments(mask)
		if M["m00"]!=0:
			cx = int(M["m10"] / M["m00"])
			cy = int(M["m01"] / M["m00"])
			cv2.circle(output, (cx, cy), 7, (255, 255, 255), -1)
			cv2.circle(output, (cx,cy), 40, 255)
			cv2.rectangle(output,(xlower,ylower),(xupper,yupper),255,2)
			cv2.arrowedLine(output,(centerx,11),(centerx,2),255,2)

			if cx < xlower or cx > xupper:
				xspeed=float(alphax*(centerx-cx)) #pos val means object is left, neg means object is right of cntr
				
				#print(xspeed)
				
				if xspeed > 0:
					xspeed=1-(1/xspeed) #normalize value between -1 and 1
				else: 
					xspeed=-1-(1/xspeed)
				
				print(xspeed)#roll speed
			else: xspeed=0
			if cy < ylower or cy > yupper:
				yspeed=float(alphay*(centery-cy)) #pos val means object is above, neg means object is below
				if yspeed > 0:
					yspeed=1-(1/yspeed) #normalize val between -1 and 1
				else:
					yspeed=-1-(1/yspeed)
				print(yspeed)#pitch speed
			else:
				yspeed=0 
#incorrect but sameform controller.SetCommand(self.roll,self.pitch,self.yaw_velocity,self.z_velocity)

		bgr_output=cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
		cv2.imshow("Processed Video", hstack([image, bgr_output]))
		

if __name__=='__main__':
	
	display=DroneVideo()
        #initialize node
        rospy.init_node('drone_video')
        rospy.spin()
        cv2.destroyAllWindows()


