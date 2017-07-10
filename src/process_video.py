#!/usr/bin/env python

import sys

#import the ROS libraries and load the manifest file which through <depend packages=/// /> will give us access to the project dependences
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

#import opencv and numpy for image processing
import cv2
import numpy
import time
from cv_bridge import CvBridge, CvBridgeError
#from drone_video import DroneVideo

class ProcessVideo(object):

    def __init__(self,cv_image):
    #   super(ProcessVideo, self).__init__()
        
        #print("done")  

        def DisplayVideo(self,cv_image):
    
            cv2.imshow("DroneVideo", self.cv_image)

        
    
#if __name__=='__main__':
    
#   DroneVideo()    
    #ProcessVideo()

    #rospy.init_node('process_video')
    #rospy.spin()

    #cv2.destroyAllWindows()
