#!/usr/bin/env python

import sys
import rospy

#import opencv and numpy for image processing
import cv2
from numpy import *
import time
from cv_bridge import CvBridge, CvBridgeError
from os.path import expanduser

# This class contains helper functions that each process
# video frames in some way
class ProcessVideo(object):


#The purpose of this function is to segement an image by setting all of the pixels an image
#outside of a specified color range to 0, only leaving the desired color in the image.

#param2::image: the image we would like to segment, expected to be bgr
#param3::color: string specifying the color you would like to keep. These values are defined below
#param4::returnType: string to specify the type of image, you would like this function to return.
#by default will return a segmented image in BGR, this can also be specified to return following.
#"binary" returns a binary image, 1 is region within color range
#"hsv" returns an hsv image, of pixels within color range

    def DetectColor(self,image,color,returnType = "segmented"):
        
        numRows,numCols,channels=image.shape
        lower2=upper2=array([0,0,0])
	#definitions for upper and lower hsv values for each color
        if(color=='orange'): #0,80,190,7,255,255
            hsv_boundaries = [( [0, 50, 170],[10, 254, 255] )]
            #lower  hsv boundary #170 140 150,179 255 255
            hsv_boundaries2 = [([172, 50, 180],[180, 254, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
            lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
            upper2=array(hsv_boundaries2[0][1], dtype = "uint8")
        if(color=='front orange'):
            hsv_boundaries = [( [0, 50, 170],[10, 254, 255] )]
            #lower  hsv boundary #170 140 150,179 255 255
            hsv_boundaries2 = [([170, 50, 180],[180, 254, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
            lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
            upper2=array(hsv_boundaries2[0][1], dtype = "uint8")

        if(color=='blue'):
            hsv_boundaries = [ ([102,110,70],[115,255,255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
        
        if(color=='green'):
            hsv_boundaries = [ ([40, 70, 0],[70, 190, 254])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")

	#convert bgr to hsv image for color segmentation
        hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #find colors within the boundaries for each pixel,returns binary image
        mask = mask1 = cv2.inRange(hsv_image,lower,upper)

        # if a second bound exists, we combine the two masks
        if( lower2.any() or upper2.any() ):
            mask2 = cv2.inRange(hsv_image,lower2,upper2)
	    #perform logical or to combine the masks
            mask = cv2.bitwise_or(mask1,mask2,mask=None)
        
	#this sets any pixel in mask != 0 to its hsv color value from unsegmented image
        hsv_output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)
        #this converts the image from hsv to bgr
        segmentedImage = cv2.cvtColor(hsv_output,cv2.COLOR_HSV2BGR)
        #we put a circle in the center of the image 
        #cv2.circle(segmentedImage,(numcols/2,numrows/2),4,150,1)
        
        #rospy.logwarn(hsv_image[numRows/2][numCols/2])
        #rospy.logwarn("seg: " + str(hsv_output[numRows/2][numCols/2]))
        #segmentedImage is bgr, and mask is a binary image with values within color range
        if(returnType == "segmented"):
            return segmentedImage
        elif returnType == "hsv":
            return hsv_output
        elif returnType == "binary":
            return mask
        elif returnType == "all":
            return segmentedImage,hsv_output,mask

    #this function takes in as arguements, object aproximate pixel length , the original size of an object in
    #mm and, focal length (default is focal length of A.R. drone bottom camera). Then it finds the approximate
    #distance in mm, from the object to the camera. (assumes flat lense)
    def CalcDistance(self,trueObjectSize,objectPixelSize,focalLength = 459.2622):
        #z = foc*x/x'
        distance = (focalLength*trueObjectSize)/objectPixelSize
        return distance

    #a non classical more accurate model for calculating distance,object true size expected in mm
    #and returns distance in mm
    def CalcDistanceNew(self,objectTrueSize,objectPixels,focalLength = 781.6, offset = -319.4):
        distance = ( (focalLength*objectTrueSize)/objectPixels)+offset
        return distance
        
    #calculates the focal length of a flat lense, given as parameters: an objects apparent size in pixels,
    #an objects true size in mm, and the distance from the object in mm
    def CalcFocal(self,objectPixelSize,objectTrueSize,distance):
        focal = (objectPixelSize*distance)/trueObjectSize
        return focal
        
    # Performs houghline transform to detect lines by inputing a BGR image and returning
    # the angle of the line and the point perpendicular to the origin
    def ShowLine(self,image, lowerAngleBound = 0, upperAngleBound = 180, secondBounds = (None,None), thresh=65):
        
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
            #rospy.logwarn("first bound:" + str(extract))

            if(secondBounds != (None,None)):
                extractSecond = logical_and((thetasDegrees > secondBounds[0]),(thetasDegrees < secondBounds[1]))
                extract = logical_or(extract,extractSecond)
            
                #rospy.logwarn("second bound:"+str(extractSecond))
            #rospy.logwarn("combined:" + str(extract))

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
    
    def ApproximateSpeed(self, image, cx, cy, currAltitude = None, desiredAltitude = None,
    xtolerance = 20, ytolerance = 20, ztolerance = 75 ):

        numrows,numcols,channels = image.shape

        centerx = numcols/2
        centery = numrows/2

        #create a "window" for desired center of mass position
        width = xtolerance * 2
        height = ytolerance * 2
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
        tolerance = ztolerance
        climbSpeed = 0.8
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
                alphax = 0.2
            else:
                alphax = 0.2
        
            # if it's out of vertical close zone
            if cy < zoneTop or cy > zoneBottom:
                alphay = 0.2
            else:
                alphay = 0.2

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
    def LineOrientation(self, image, angle, thresh, yawspeed = 0.4):

        if angle == None:
            return None

        numrows, numcols, _ = image.shape
        centerx = numcols/2
        centery = numrows/2
        upperAngle = 90 + thresh
        lowerAngle = 90 - thresh

        # Drone rotates Counter Clock_Wise
        if angle < lowerAngle and angle > 0:
            yawSpeed = yawspeed

        # Drone rotates Clock_Wise
        elif angle > upperAngle and angle < 180:
            yawSpeed = -1 * yawspeed

        # Drone is at the right angle; no need to rotate 
        else:
            yawSpeed = 0
        
        return yawSpeed
    

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
            yawspeed = 0.4

        # Drone rotates Clock_Wise
        elif angle > lowerAngle and angle < 90:
            yawspeed = -0.4

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

    #given an image and a color of a circle, this function will segment the image according to the
    #desired color and return the first circular object that it sees, along with its 
    #calculated radius and center, if no circle is detected, these two values will be == None
    def DetectCircle(self,image, circleColor = None):
        numrows,numcols,channels=image.shape
        imagePerimeter = 2*numrows+2*numcols
        minCircumference = 0.01*imagePerimeter
        minR = int(minCircumference/(2*pi))
        #first segment the image by color of circle
        if circleColor != None:
            segmentedImage = self.DetectColor(image, circleColor)
            grayImage = cv2.cvtColor(segmentedImage,cv2.COLOR_BGR2GRAY)
        else:
            grayImage = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            segmentedImage = grayImage
            
        grayImage = cv2.GaussianBlur( grayImage, (7,7),0)
        circles = cv2.HoughCircles(grayImage,cv2.HOUGH_GRADIENT,1,10,param1=50,param2=20,minRadius=minR,maxRadius=0)
        if circles != None:
            for param in circles[0,:]:
                cx=int(param[0])
                cy=int(param[1])
                center = (cx,cy)
                radius = param[2]
                # draw the outer circle
                cv2.circle(segmentedImage,center,int(radius),(0,255,0),2)
                # draw the center of the circle
                cv2.circle(segmentedImage,center,2,(0,0,255),3)
                return segmentedImage, radius, center
        #if we have looped through every object and dont see a circle, return None
        return image, None, None

    def DetectShape(self,image, circleColor):
        #bottom camera f = 408.0038
        #first segment the image by color of circle
        segmentedImage,_,binaryImage = self.DetectColor(image, circleColor,"all")
        numrows,numcols,channels=segmentedImage.shape
        imagePerimeter = 2*numrows+2*numcols
        grayImage = cv2.cvtColor(segmentedImage,cv2.COLOR_BGR2GRAY)
        grayImage = cv2.GaussianBlur( grayImage, (7,7),0)

        contours = cv2.findContours(grayImage.copy() , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #use contours[0] for opencv2 or contours[1] for opencv3
        contours = contours[1]
        for shape in contours:
            perimeter = cv2.arcLength(shape,True)
            #if the shape is too small, it is most likely noise, and we wish to disregard it
            if perimeter > (0.05)*imagePerimeter:
            #finds shapes that are within a certain percentage of original shape perimeter
                vertices = cv2.approxPolyDP(shape, 0.009 * perimeter,True)
                numVertices = len(vertices)
                #the shape is determined by the number of vertices,i.e. a triangle has 3, square has 4, 
                #pentagon has 5, and anything above will be considered circular.
                if numVertices > 5:
                    M = cv2.moments(vertices)
                    if(M["m00"] != 0):
                        cx = int(M["m10"] / M["m00"])
                        cy= int(M["m01"] / M["m00"])
                        #center of circle
                        center = (cx,cy)
                        numPoints = 0
                        averageRadius = 0
                        #we want to loop through every vertex on circle and measure distance to center
                        for points in vertices:
                            point = points[0]
                            #this will check if the circle is being cut off by image boundary
                            if(point[0] < 5 or point[0] >= numcols-5 or point[1] < 5 or point[1] >= numrows-5):
                                return image,None,None
                            else:
                                dist = (point - center)
                                currentRadius = sqrt(inner(dist,dist))
                                averageRadius += currentRadius
                                numPoints += 1
                        #we want to calculate the average radius and return it as # of pixels
                        averageRadius = (averageRadius/numPoints)
                        
                        #draw circle onto image
                        cv2.circle(segmentedImage, center,1,(255,255,255),-1)
                        cv2.circle(segmentedImage,center, int(averageRadius),(255,255,255),2)
                        #cv2.drawContours(segmentedImage,[vertices],-1,(0,255,0),2)
                        #this will return after the first circle is detected
                        return segmentedImage, averageRadius, center
        #if we have looped through every object and dont see a circle, return None
        return image, None, None

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


    def DetectFaces(self,image):
        cascPath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/haarcascade_frontalface_default.xml"

        # Create the haar cascade
        faceCascade = cv2.CascadeClassifier(cascPath)

        # Convert image to gray
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=5,
        minSize=(30, 30),
        flags = cv2.CASCADE_SCALE_IMAGE)

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cx = (2*x + w)/2
            cy = (2*y + w)/2
            return cx,cy

