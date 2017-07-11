#!/usr/bin/env python

import sys
import rospy

#Import messages we want for recieving video feed
from sensor_msgs.msg import Image 
from ardrone_autonomy.msg import Navdata

#Import service type for toggle cam
from std_srvs.srv import Empty

# Import cv2 as GUI
import cv2
from cv_bridge import CvBridge, CvBridgeError


class DroneVideo(object):

    def __init__(self):

        self.bridge=CvBridge()
        self.cv_image=None

        #Subscribe to the drones video feed
        self.video=rospy.Subscriber('/ardrone/image_raw', Image, self.ROStoCVImage )

        #initialization for recording video
        #fourcc=cv2.VideoWriter_fourcc(*'XVID')
        #self.out=cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))      

    def ROStoCVImage(self,data):
        
        #convert ROS Image to OpenCV Image
        self.cv_image=self.bridge.imgmsg_to_cv2(data, "bgr8")

        #get size of image
        height, width, channels = self.cv_image.shape
        size = self.cv_image.size
        self.ShowVideo()
        

    def ShowVideo(self):

        #processing goes here 
        cv2.imshow("video", self.cv_image)
        cv2.waitKey(3)
        
    
if __name__=='__main__':
    
    display=DroneVideo()
    rospy.init_node('DroneVideo')
    rospy.spin()
cv2.destroyAllWindows()
