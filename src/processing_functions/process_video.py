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

        #find the outline of all orange objects in image
	#set any pixel != 0 to its original color value from unsegented image
        output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)
        output = cv2.cvtColor(output,cv2.COLOR_HSV2BGR)
        #return the segmented image
        #show the contours on original image
        #self.FindBox(output,mask)
        
        return output

    #performs houghline transform to detect lines by inputing a BGR image and returning the angle of the line and the point perpendicular to the origin
    def ShowLine(self,image):
        
        #change bgr to gray for edge detection
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #now we have binary image with edges of tape
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        #lines contains rho and theta values
        
        thresh=100
        lines = cv2.HoughLines(edges,1,pi/180,thresh)
        while(type(lines)==type(None) and thresh > 0):
            lines = cv2.HoughLines(edges,1,pi/180,thresh)
            thresh-=thresh
        
        #average rho and theta values
        if(thresh!=0):
            #rospy.logwarn(lines)
            LINES = matrix(lines).mean(0)
            rho=LINES[0,0]
            radians=LINES[0,1]
        
            a = cos(radians)
            b = sin(radians)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
    
            cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)
            cv2.circle(image,(x0,y0),255,-1)
            #this is the correct angle relative to standard cordinate system for average line
            angle=(-1*((radians*180)/pi)+90)
        
        else:
            x0=None
            y0=None
            angle=None

        return (x0,y0,angle)
        
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
            #Draw a circle only when an object is detected
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 
            cv2.circle(image, (cx,cy), 40, 255)

	else:
            #Set Cx and Cy to zero
	    cx=centerx
	    cy=centery

        return (cx,cy)
    
    #takes in an image and the center of masses for its segmented version, 
    #returns how much the drone should move in the (x,y) direction such that oject stay in middle
    def ApproximateSpeed(self, image, cx, cy):
        numcols = len(image)
        numrows = len(image[0])
        centerx=numrows/2
        centery=numcols/2

        #create a "window" for desired center of mass position
        width=35
        height=35
        xlower=centerx-width #left xvalue
        ylower=centery-height #"top" yvalue
        xupper=centerx+width #right xvalue
        yupper=centery+height #"bottom" yvalue
        cv2.rectangle(image, (xlower, ylower), (xupper, yupper), (255,255,255), 3)

        # alpha changes depending on if (cx,cy) is in blue box or not
        zoneLeft = centerx - centerx/2
        zoneRight = centerx + centerx/2
        zoneTop= centery - centery/2
        zoneBottom= centery + centery/2
        cv2.rectangle(image, (zoneLeft, zoneTop), (zoneRight, zoneBottom), (255,0,0), 2)
        # if it's out of horizontal close zone
        if cx < zoneLeft or cx > zoneRight:
            #rospy.logwarn("high horizontal")
            alphax = 0.3
        else:
            #rospy.logwarn("low horizontal")
            alphax = 0.3
        
        # if it's out of vertical close zone
        if cy < zoneTop or cy > zoneBottom:
            #rospy.logwarn("high vertical")
            alphay = 0.3
        else:
            #rospy.logwarn("low vertical")
            alphay = 0.3

       #calculate movement command values for moving up, down, left, right. normalized between -1:1.
       #if object is in desired area do not move (xspeed, yspeed == 0)
        
        if cx < xlower or cx > xupper:
            #pos val means object is left, neg means object is right of cntr
            xspeed=(centerx-cx)/float(centerx)
            xspeed=alphax*xspeed
            
        else:
            xspeed=0
        
        if cy < ylower or cy > yupper:
            #pos val means object is above, neg means object is below
            yspeed=(centery-cy)/float(centery)
            yspeed=alphay*yspeed
        else:
            yspeed=0
            
        
        #draw the command speed as a vector point to center for visualization purposes
        dx=int((-100*xspeed)+centerx)
        dy=int((-100*yspeed)+centery)
        cv2.arrowedLine(image,(dx,dy),(centerx,centery),(255,0,0),3)

        return (xspeed,yspeed)
    

    """def getZone(self, imageWidth, imageHeight, cx, cy):
        
        # % gap between zones
        zoneGap = 25

        centerX=imageWidth/2
        centerY=imageHeight/2

        if cx > (centerX - (zoneGap/100.0)*imageWidth) and cx < (centerX + (zoneGap/100.0)*imageWidth) and cy """


        

    # given a segmented image hsvImage and a percentThreshold of 
    # what percentage of that image should be between hues hueMin and hueMax,
    # returns a boolean of whether or not the hsvImage and has enough of that
    # hue in it to pass that threshold 
    def isHueDominant(self, hsvImage, hueMin, hueMax, percentThreshold):
        hsvChannels = cv2.split(hsvImage)
        hue = hsvChannels[0]

        percentOrange = float((len(hsvImage)*len(hsvImage[0]))-count_nonzero(hue<hueMax)) / (len(hsvImage)*len(hsvImage[0]) )

        percentOrange = percentOrange * 100

        if percentOrange > percentThreshold:
            return True
        else:
            return False

    def FindBox(self,image,mask):
        im2, contours,hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours=contours[0][:][:]
        mx=amax(contours,0)
        mn=amin(contours,0)
        maxx=mx[0][0]
        maxy=mx[0][1]
        minx=mn[0][0]
        miny=mn[0][1]
        #only draw from the outer edges of square
        contours=np.array([[[minx,miny],[minx,maxy]],[[minx,miny],[maxx,miny]],
        [[maxx,miny],[maxx,maxy]],[[maxx,maxy],[minx,maxy]]])
        #draw the square on the original image
        cv2.drawContours(image, contours, -1, (0,255,0), 3)


