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

imagePath = sys.argv[1]
image = cv2.imread(imagePath)

process= ProcessVideo()

orangeImage,radius,center = process.DetectCircle(image,"orange")

cv2.imshow("processed",orangeImage)
#cv2.imshow("orig",image)
print(str(radius*2))
cv2.waitKey(0)

