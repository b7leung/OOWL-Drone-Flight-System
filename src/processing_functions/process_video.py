#!/usr/bin/env python

import sys
import rospy

#import opencv and numpy for image processing
import cv2
from numpy import *
import time
from cv_bridge import CvBridge, CvBridgeError


# This class contains helper functions that each process
# video frames in some way
class ProcessVideo(object):


    #returns segemented image that only leaves a pixels within specified color value, sets else to 0
    def DetectColor(self,image,color):
        
        numrows,numcols,channels=image.shape

	#upper orange hsv boundary
        if(color=='orange'):
            hsv_boundaries = [ ([0, 80, 190],[7, 255, 255])]
            #lower  hsv boundary
            hsv_boundaries2 = [([170, 140, 150],[179, 255, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
            lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
            upper2=array(hsv_boundaries2[0][1], dtype = "uint8")

        if(color=='blue'):
            hsv_boundaries = [ ([102,110,70],[115,255,255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
            lower2=lower
            upper2=upper
        
        if(color=='green'):
            hsv_boundaries = [ ([40, 70, 0],[70, 190, 254])]
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
        hsv_output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)
        rospy.logwarn(hsv_output[numrows/2][numcols/2])
        rospy.logwarn(hsv_image[numrows/2][numcols/2])
        output = cv2.cvtColor(hsv_output,cv2.COLOR_HSV2BGR)
        cv2.circle(output,(numcols/2,numrows/2),4,150,1)
        #return the segmented image
        #show the contours on original image
        
        return output
        

    # Performs houghline transform to detect lines by inputing a BGR image and returning
    # the angle of the line and the point perpendicular to the origin
    def ShowLine(self,image, lowerAngleBound = 0, upperAngleBound = 180, thresh=65):
        
        #change bgr to gray for edge detection
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #now we have binary image with edges of tape
        gray = cv2.GaussianBlur( gray, (7,7),0)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        #lines contains rho and theta values
        
        lines = cv2.HoughLines(edges,1,pi/180,thresh)
        """while(type(lines)==type(None) and thresh > 30):
            lines = cv2.HoughLines(edges,1,pi/180,thresh)
            thresh-=1
       """
        #average rho and theta values
        if(lines!= None):
        
            Lines=array(lines)
            thetas=Lines[:,:,1]
            rhos=Lines[:,:,0]
            thetasDegrees = (thetas*180)/pi

            #rospy.logwarn(thetasDegrees)

            #boolean arrays for angles greater than 170 and less than 10
            large = (thetasDegrees>170)
            small = (thetasDegrees<10)
            extract = logical_and((thetasDegrees > lowerAngleBound),(thetasDegrees < upperAngleBound))
            #if most lines are within this range of theta

            if( sum(large | small) > (size(thetas)/2) and (lowerAngleBound <= 0)):
                if(sum(large) > sum(small)):
                    #sums up all angles greater than 170 and averages them
                    radians = sum(thetas*large)/(sum(large))
                    rho = sum(rhos*large)/(sum(large))

                    """rospy.logwarn((radians*180)/pi)
                    rospy.logwarn(rho)
                    rospy.logwarn("size")

                    rospy.logwarn(thetas)"""


                else:
                    #adds up all angles less than 10 and averages them
                    radians = sum(thetas*small)/(sum(small))
                    rho = sum(rhos*small)/(sum(small))
                    """rospy.logwarn((radians*180)/pi)
                    rospy.logwarn(rho)
                    rospy.logwarn("size")

                    rospy.logwarn(thetas)"""

            else:
                #takes average of all angles
                temp=(thetas*extract)*180/pi
                #rospy.logwarn(temp)
                radians = sum(thetas*extract)/(sum(extract))
                rho = sum(rhos*extract)/(sum(extract))
                

                #LINES = matrix(lines).mean(0)
                #rho=LINES[0,0]
                #radians=LINES[0,1]
            # all data needed to plot lines on original image
            
            if( not math.isnan(radians)):
                a = cos(radians)
                b = sin(radians)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
    
                cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)
            #this is the correct angle relative to standard cordinate system for average line
                angle=((radians*180)/pi)
                radians=radians/pi
                #rospy.logwarn(angle)

            else: 
                angle = None
        else:
            x0=None
            y0=None
            angle=None
            rho=None
            radians=None

        return angle


    # Takes in a segmented image input and returns a tuple in the form (x,y,bool),
    # where x and y are the center of mass and bool is whether an object was found
    # If it does not exist, the center of mass is set to the middle of the screen
    def CenterOfMass(self,image):

        numrows,numcols,channels=image.shape

        centerx=numcols/2
        centery=numrows/2

        #compute the center of moments for a single-channel gray image
        M=cv2.moments(cv2.cvtColor(cv2.cvtColor(image,cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY),True) 

        if M["m00"]!=0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            #draw circle on cx and cy only if center of mass exists
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 
            cv2.circle(image, (cx,cy), 40, 255)
        
            return(cx,cy)

	else:
            # if center of mass doesn't exist; set them as the center
            return (None, None)


    # takes in an image and the center of masses for its segmented version, 
    # returns how much the drone should move in the (x,y) direction such that
    # object stays in middle, within +/- tolerance pixels of the center
    
    def ApproximateSpeed(self, image, cx, cy, currAltitude = None, desiredAltitude = None, tolerance = 20 ):

        numrows,numcols,channels = image.shape

        centerx = numcols/2
        centery = numrows/2

        #create a "window" for desired center of mass position
        width = tolerance * 2
        height = tolerance * 2
        xlower = centerx-width #left xvalue
        ylower = centery-height #"top" yvalue
        xupper = centerx+width #right xvalue
        yupper = centery+height #"bottom" yvalue

        # alpha changes depending on if (cx,cy) is in blue box or not
        zoneLeft = centerx - centerx/2
        zoneRight = centerx + centerx/2
        zoneTop = centery - centery/2
        zoneBottom = centery + centery/2
        
        
         # calculating if the drone should go up or down to match the desired altitude
        tolerance = 200
        climbSpeed = 0.25
        if currAltitude != None and desiredAltitude != None:
            if (currAltitude < (desiredAltitude - tolerance)):
                zVelocity = climbSpeed
            elif (currAltitude > (desiredAltitude + tolerance)):
                zVelocity = climbSpeed * -1
            else:
                zVelocity = 0
        else:
            zVelocity = 0
            
        #Draws a rectangle for the center part of the drone desired to hover over object
        cv2.rectangle(image, (xlower, ylower), (xupper, yupper), (255,255,255), 3)
        #cv2.rectangle(image, (zoneLeft, zoneTop), (zoneRight, zoneBottom), (255,0,0), 2)

    
        #draw circle and calculate speeds only if the center of mass for object exists
        if (cx != None) and (cy != None):
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 
            cv2.circle(image, (cx,cy), 40, 255)

            # if it's out of horizontal close zone
            if cx < zoneLeft or cx > zoneRight:
                alphax = 0.35
            else:
                alphax = 0.35
        
            # if it's out of vertical close zone
            if cy < zoneTop or cy > zoneBottom:
                alphay = 0.35
            else:
                alphay = 0.35

       #calculate movement command values for moving up, down, left, right. normalized between -1:1.
       #if object is in desired area do not move (xspeed, yspeed == 0)
        
            if (cx < xlower) or (cx > xupper):
                #pos val means object is left, neg means object is right of cntr
                xspeed = (centerx-cx)/float(centerx)
                xspeed = alphax*xspeed
            
            else:
                xspeed = 0
        
            if (cy < ylower) or (cy > yupper):
                #pos val means object is above, neg means object is below
                yspeed = (centery-cy)/float(centery)
                yspeed = alphay*yspeed
            else:
                yspeed = 0
        else: 
            xspeed = 0
            yspeed = 0

        #draw the command speed as a vector point to center for visualization purposes
        dx=int((-100*xspeed)+centerx)
        dy=int((-100*yspeed)+centery)
        cv2.arrowedLine(image,(dx,dy),(centerx,centery),(255,0,0),3)

        return (xspeed, yspeed, zVelocity)


    # returns yawspeed, keeps blue line horizontal in bottom cam, yspeed keeps line in middle
    # keep blue line between +/- thresh of 90 degrees (considered perfect at 90 degrees)
    # returns None if no yawspeed could be calculated
    def LineOrientation(self, image, angle, thresh):

        if angle == None:
            return None

        numrows, numcols, _ = image.shape
        centerx = numcols/2
        centery = numrows/2
        upperAngle = 90 + thresh
        lowerAngle = 90 - thresh

        # Drone rotates Counter Clock_Wise
        if angle < lowerAngle and angle > 0:
            yawspeed = 1

        # Drone rotates Clock_Wise
        elif angle > upperAngle and angle < 180:
            yawspeed = -1

        # Drone is at the right angle; no need to rotate 
        else:
            yawspeed = 0
        
        return yawspeed
    

    # return yawspeed keeps blue line vertical in bottom cam, xspeed keeps line in middle
    # keeps line between +/- thresh from 0 degrees (perfect at 0 degrees)
    # returns None if no yawspeed could be calculated
    def ObjectOrientation(self, image, angle, thresh):
        
        if angle == None:
            return None

        numrows, numcols, _ = image.shape
        centerx = numcols/2
        centery = numrows/2
        upperAngle = 180 - thresh
        lowerAngle = 0 + thresh
        
        # Drone rotates Counter Clock-Wise
        if angle < upperAngle and angle > 90:
            yawspeed = 0.5

        # Drone rotates Clock_Wise
        elif angle > lowerAngle and angle < 90:
            yawspeed = -0.5

        else:
            yawspeed = 0

        return yawspeed
        

    # given a segmented image hsvImage and a percentThreshold of 
    # what percentage of that image should be between hues hueMin and hueMax,
    # returns a boolean of whether or not the hsvImage and has enough of that
    # hue in it to pass that threshold 
    def IsHueDominant(self, hsvImage, hueMin, hueMax, percentThreshold):
        
        # getting array of image that considers only hue, not saturation nor value
        hsvChannels = cv2.split(hsvImage)
        hue = hsvChannels[0]
        
        # find ratio of pixels whose hue is within range, to the number of pixels overall in image
        numHuePixel = float( count_nonzero( (hueMin<hue) & (hue<hueMax) ) )
        numImagePixel= (len(hsvImage)*len(hsvImage[0]) )
        hueRatio = numHuePixel / numImagePixel

        huePercent = hueRatio * 100
        if huePercent > percentThreshold:
            return True
        else:
            return False


    # given a segmented hsv image of only 1 color, will attempt to return that
    # image with unwanted static/noise reduced
    def DeNoiseImage(self, hsvImage):
        #dst = cv2.fastNlMeansDenoisingColored(hsvImage, None, 10,10,7,5)
        dst = None
        cv2.medianBlur(hsvImage,5, dst)
        if dst == None:
            rospy.logwarn("none")
        else:
            rospy.logwarn("not none")

        return dst
    #takes in an image and the center coordinate as a tuple and simply draws a circle over the coordinate    
    def DrawCircle(self,image,center):
    
        cx = center[0]
        cy = center[1]

        
        #draw circle on cx and cy only if center is valid
        if cx != None and cy != None:
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 
            cv2.circle(image, (cx,cy), 40, 255)


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


    # Given an image, a point (x,y), and a width/height,
    # Will return a "cropped image" of the same dimensions
    # where only a box whose upper left corner starts at (x,y) 
    # and a corresponding dimentions of width/height will correspond
    # to visible image. The rest of the image will be black.
    def CropVisible(self, image, x, y, width, height):
        
        # first, make 0 filled array of the same shape as the original image
        rect_image = zeros( (len(image),len(image[0]),len(image[0][0])), uint8 )
        # move each pixel that fits the (x,y) and width height criteria from image to the empty
        # array
        rect_image[y:y+height,x:x+width, 0:3:1] = image[y:y+height,x:x+width, 0:3:1]
        return rect_image 
                    
