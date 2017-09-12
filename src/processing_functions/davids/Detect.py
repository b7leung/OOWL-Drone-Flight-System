#!/usr/bin/env python

import sys
import rospy

#import opencv and numpy for image processing
import cv2
from numpy import *
import time
from cv_bridge import CvBridge, CvBridgeError
from os.path import expanduser

from process_video import ProcessVideo


process = ProcessVideo()

imagePath = sys.argv[1]
image = cv2.imread(imagePath)
orangeImage,radius,center = process.DetectShape(image,"front orange",.3)
cv2.imshow("image",orangeImage)
print(str([center[1]])+" "+str([center[0]]))
print(str(radius) )
cv2.waitKey(0)


