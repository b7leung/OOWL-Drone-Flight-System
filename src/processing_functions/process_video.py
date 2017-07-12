#!/usr/bin/env python

import sys
import rospy

#import opencv and numpy for image processing
import cv2
import numpy
import time
from cv_bridge import CvBridge, CvBridgeError

class ProcessVideo(DroneVideo):

    def DetectColor(self,image):
        
# define the list of boundaries (order is BGR values)
#starts with detecting red, blue yellow and then gray
        
        
	#orange hsv boundary
        hsv_boundaries = [
            ([0, 150, 200],[7, 255, 255])
            ]
	#second hsv boundary
        hsv_boundaries2 = [([170, 140, 150],[179, 255, 255])
            ]
	
	#convert bgr to hsv image for color segmentation
        hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	#create numpy arrays from boundaries of the color
        lower= array(hsv_boundaries[0][0],dtype = "uint8")
        upper= array(hsv_boundaries[0][1],dtype = "uint8")
        lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
        upper2=array(hsv_boundaries2[0][1], dtype = "uint8")

        #find colors within the boundaries for each color set=1, else=0
        mask1 = cv2.inRange(hsv_image,lower,upper)
        mask2 = cv2.inRange(hsv_image,lower2,upper2)
	#perform logical or to combine the masks
        mask = cv2.bitwise_or(mask1,mask2,mask=None)
	#set any pixel != 0 to its original color value from unsegented image
        output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)
        return output #return the segmented image

    def ShowLine(self,image):
        numcols = len(image)
        numrows = len(image[0])
        #preprocess image to make it easier to see lines
        image = self.DetectColor(image) #segment color of tape
        image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR) #change hsv to bgr
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) #change bgr to gray for edge detection
        edges = cv2.Canny(gray,50,150,apertureSize = 3) #now we have binary image with edges of tape
        lines = cv2.HoughLines(edges,1,np.pi/360,100) #lines contains rho and theta values
        LINES = np.matrix(lines).mean(0)#average rho and theta values
        rho=LINES[0,0]
        degrees=LINES[0,1]
        
        a = np.cos(degrees)
        b = np.sin(degrees)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
    
        cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2) #"average fit line"
        angle=(-1*((degrees*180)/np.pi)+90)#this is the correct angle relative to standard cordinate system for average line

        return image
        
    def CenterofMass(self,image):
        numcols = len(image)
        numrows = len(image[0])
        centerx=numrows/2
        centery=numcols/2

        M=cv2.moments(cv2.cvtColor(cv2.cvtColor(image,cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY),True) 
        if M["m00"]!=0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
	else:
	    cx=centerx
	    cy=centery

    def ApproximateSpeed(self, image, cx, cy):
        numcols = len(image)
        numrows = len(image[0])
        centerx=numrows/2
        centery=numcols/2

        xlower=centerx-60 #left xvalue
        ylower=centery-60 #"top" yvalue
        xupper=centerx+60 #right xvalue
        yupper=centery+60 #"bottom" yvalue
        alphax=.03
        alphay=.02
    
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
    
    

