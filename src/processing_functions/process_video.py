#!/usr/bin/env python

import sys
import rospy

#import opencv and numpy for image processing
import cv2
from numpy import *
import time
from cv_bridge import CvBridge, CvBridgeError

class ProcessVideo(object):

    #returns segemented image that only leaves a pixels within specified color value, sets else to 0
    def DetectColor(self,image,color):
          
	#upper orange hsv boundary
        if(color=='orange'):
                hsv_boundaries = [ ([0, 150, 200],[7, 255, 255])]
	        #lower  hsv boundary
                hsv_boundaries2 = [([170, 140, 150],[179, 255, 255])]
                lower=array(hsv_boundaries[0][0], dtype = "uint8")
                upper= array(hsv_boundaries[0][1],dtype = "uint8")
                lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
                upper2=array(hsv_boundaries2[0][1], dtype = "uint8")

        if(color=='blue'):
                 hsv_boundaries = [ ([100,160,100],[115,255,255])]
                 lower=array(hsv_boundaries[0][0], dtype = "uint8")
                 upper= array(hsv_boundaries[0][1],dtype = "uint8")
                 lower2=lower
                 upper2=upper

	#convert bgr to hsv image for color segmentation
        hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #find colors within the boundaries for each color set=1, else=0
        mask1 = cv2.inRange(hsv_image,lower,upper)
        mask2 = cv2.inRange(hsv_image,lower2,upper2)
	#perform logical or to combine the masks
        mask = cv2.bitwise_or(mask1,mask2,mask=None)
	#set any pixel != 0 to its original color value from unsegented image
        output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)
        output = cv2.cvtColor(output,cv2.COLOR_HSV2BGR)
        return output #return the segmented image

    def ShowLine(self,image):
        numcols = len(image)
        numrows = len(image[0])
        #preprocess image to make it easier to see lines
        #segment color of tape
        
        image = self.DetectColor(image,'blue')
        original=image
        #change hsv to bgr
        #image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
        #change bgr to gray for edge detection
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #now we have binary image with edges of tape
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        #lines contains rho and theta values
        
        thresh=100
        lines = cv2.HoughLines(edges,1,pi/360,thresh)
        while(type(lines)==type(None) and thresh > 0):
            lines = cv2.HoughLines(edges,1,pi/360,thresh)
            thresh-=thresh
        
        #average rho and theta values
        if(thresh!=0):
            rospy.logwarn(lines)
            LINES = matrix(lines).mean(0)
            rospy.logwarn(LINES)
            rho=LINES[0,0]
            degrees=LINES[0,1]
        
            a = cos(degrees)
            b = sin(degrees)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
    
            #"average fit line"
            cv2.line(original,(x1,y1),(x2,y2),(0,0,255),2)
            #this is the correct angle relative to standard cordinate system for average line
            angle=(-1*((degrees*180)/pi)+90)
        return original
        
    #takes in a segmented image input and returns the center of mass in x and y coordinates
    def CenterofMass(self,image):
        numcols = len(image)
        numrows = len(image[0])
        centerx=numrows/2
        centery=numcols/2
        
        #compute the center of moments for a single-channel gray image
        #if cx and cy DNE then set valuews such that xspeed and yspeed == 0
        M=cv2.moments(cv2.cvtColor(cv2.cvtColor(image,cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY),True) 
        if M["m00"]!=0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
	else:
	    cx=centerx
	    cy=centery
        
        #draw a circle on the center of mass on the segmented image and track it
        cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1)
        cv2.circle(image, (cx,cy), 40, 255)
        cv2.arrowedLine(image,(centerx,11),(centery,2),255,2)

        return (cx,cy)
    
    #takes in an image and the center of masses for its segmented version, 
    #returns how much the drone should move in the (x,y) direction such that oject stay in middle
    def ApproximateSpeed(self, image, cx, cy):
        numcols = len(image)
        numrows = len(image[0])
        centerx=numrows/2
        centery=numcols/2

        #create a "window" for desired center of mass position
        xlower=centerx-60 #left xvalue
        ylower=centery-60 #"top" yvalue
        xupper=centerx+60 #right xvalue
        yupper=centery+60 #"bottom" yvalue
        alphax=.03
        alphay=.02
    

        #calculate movement command values for moving up, down, left, right. normalized between -1:1.
       #if object is in desired area do not move (xspeed, yspeed == 0)
        
        if cx < xlower or cx > xupper:
            #pos val means object is left, neg means object is right of cntr
            xspeed=(centerx-cx)/float(centerx)              
        else:
            xspeed=0
        
        if cy < ylower or cy > yupper:
            #pos val means object is above, neg means object is below
            yspeed=(centery-cy)/float(centery)
        else:
            yspeed=0

        return (xspeed,yspeed)

        #non-linear way to normalize value between -1 and 1    
        #if xspeed > 0:
            #xspeed=1-(1/xspeed)                 
        #else: 
            #xspeed=-1-(1/xspeed)
                
            

        #if yspeed > 0:
            #yspeed=1-(1/yspeed) #normalize val between -1 and 1
        #else:
            #yspeed=-1-(1/yspeed)
    
    
    

