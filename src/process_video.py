#!/usr/bin/env python

import sys

import rospy

#import opencv and numpy for image processing
import cv2
import numpy
import time
from cv_bridge import CvBridge, CvBridgeError
from drone_video import DroneVideo

class ProcessVideo(DroneVideo):

    def __init__(self,cv_image):


    def DetectColor(self,image):
        
        # define the list of boundaries (order is BGR values)
        #starts with detecting red, blue yellow and then gray
        
        #bgr_boundaries = [
        #   ([0,18,220],[150,150,255])
        #   ]
        
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
        self.output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)

    def ApproximateSpeed(self):
        numcols = len(self.output)
        numrows = len(self.output[0])
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
            cv2.circle(self.output, (cx, cy), 7, (255, 255, 255), -1)
            cv2.circle(self.output, (cx,cy), 40, 255)
            cv2.rectangle(self.output,(xlower,ylower),(xupper,yupper),255,2)
            cv2.arrowedLine(self.output,(centerx,11),(centerx,2),255,2)

            if cx < xlower or cx > xupper:
                #pos val means object is left, neg means object is right of cntr
                xspeed=(centerx-cx)/float(centerx)              
                
                #if xspeed > 0:
                    #xspeed=1-(1/xspeed) #normalize value between -1 and 1
                #else: 
                    #xspeed=-1-(1/xspeed)
                
                print("xspeed",xspeed)#roll speed
            
            else: xspeed=0
            if cy < ylower or cy > yupper:
                #pos val means object is above, neg means object is below
                yspeed=(centery-cy)/float(centery)              #if yspeed > 0:
                    #yspeed=1-(1/yspeed) #normalize val between -1 and 1
                #else:
                    #yspeed=-1-(1/yspeed)
                print("yspeed",yspeed)#pitch speed
            else:
                yspeed=0
        return (xspeed,yspeed)
    
#if __name__=='__main__':
    

