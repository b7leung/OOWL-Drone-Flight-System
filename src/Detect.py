#!/usr/bin/env python

import sys
import rospy
import ProcessVideo
#import opencv and numpy for image processing
import cv2
from numpy import *
import time
from cv_bridge import CvBridge, CvBridgeError
from os.path import expanduser

imagePath = sys.argv[1]
image = c2.imread(imagePath)
numrows,numcols,channels = image.shape
process = ProcessVideo()

orangeImage,radius,center = process.DetectCircle(image,"orange")
diam = radius*2
print(imagePath)
print("size:"+numcols +" "+numrows)
print("diam:"+diam)

