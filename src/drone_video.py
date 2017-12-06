#!/usr/bin/env python

import sys
import rospy
import numpy as np

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
        
        super(DroneVideo, self).__init__()

        self.bridge=CvBridge()
        self.cv_image=None
        self.windowName = "Live AR.Drone Video Stream"
        self.infoWindow = "Flight Info"
        self.info = np.zeros((70,100,3), np.uint8)

        #Subscribe to the drones video feed
        self.video=rospy.Subscriber('/ardrone/image_raw', Image, self.ROStoCVImage )

        self.moved = False


    def ROStoCVImage(self,data):
        
        #convert ROS Image to OpenCV Image
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.ShowVideo()


    def ShowVideo(self):
        
        self.KeyListener()

        self.ReceivedVideo()

    
        cv2.imshow(self.windowName, self.cv_image)
        cv2.imshow(self.infoWindow, self.info)
        cv2.waitKey(3)

        # move the GUI into the middle of the screenbefore the first frame is shown
        if self.moved == False :
            cv2.moveWindow(self.windowName, 750, 500)
            cv2.moveWindow(self.infoWindow, 750, 500)
            self.moved = True
        

    # processes the video before it is shown
    # don't implement here; implement in subclasses (TraceCircleController)
    def ReceivedVideo(self):
        pass

    #defines any keys to listen to
    # don't implement here; implement in subclasses (TraceCircleController)
    def KeyListener(self):
        pass
    
if __name__=='__main__':
    
    display=DroneVideo()
    rospy.init_node('DroneVideo')
    rospy.spin()
cv2.destroyAllWindows()
