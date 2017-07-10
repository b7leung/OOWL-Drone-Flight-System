import numpy as np
import argparse
import cv2
from numpy import array
import imutils
ap=argparse.ArgumentParser()
ap.add_argument("-i","--image", help = "path to the image")
args = vars(ap.parse_args())

image = cv2.imread(args["image"])
# define the list of boundaries (order is BGR values)
#starts with detecting red, blue yellow and then gray
boundaries = [
    ([17, 15, 100], [50, 56, 200]),
    ([86,0, 0], [220, 130, 50]),
    ([25, 146, 190], [62, 174, 250]),
    ([103, 86, 65], [145, 133, 128]),
    ([0,20,89],[100,200,255])
]
    
    #old blue ([86, 31, 4], [220, 88, 50]),
#loop over 3 different boundaries
for (lower,upper) in boundaries:
    #create numpy arrays from these colors
    lower= np.array(lower,dtype = "uint8")
    upper= np.array(upper, dtype = "uint8")

    #find colors within the boundaries for each color
    mask = cv2.inRange(image,lower,upper)
    output = cv2.bitwise_and(image,image, mask = mask)
    #apply a mask to the colors
    #show the images
    #cv2.imshow("images",np.hstack([image, output]))
    
    numcols = len(mask)
    numrows = len(mask[0])
    print(numrows)
    print(numcols)
    #cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #cnts = cnts[0] if imutils.is_cv2() else cnts[1]

#   loop over the contours
    #for c in cnts:
    # compute the center of the contour
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(output, (cx, cy), 7, (255, 255, 255), -1)
        cv2.circle(output, (cx,cy), 40, 255)
        cv2.rectangle(output,(numrows/2-20,numcols/2-20),(numrows/2+20,numcols/2+20),255,1)

#   samples = 0
#   centerofmass =array([[0,0]])
#   for i in range(numrows):
#       for j in range(numcols):
#           if mask[i][j]==255:
#               centerofmass[0][0] = centerofmass[0][0]+j
#               centerofmass[0][1] = centerofmass[0][1]+i
#               samples = samples+1
            #print(centerofmass)
#   if samples != 0:
#       centerofmass[0][0]=int(centerofmass[0][0]/samples)
#       centerofmass[0][1]=int(centerofmass[0][1]/samples)
#   Center=tuple(([centerofmass[0][0],centerofmass[0][1]]))
    cv2.imshow("images",np.hstack([image,  output]))    
    #cv2.circle(output, Center,40,255)   
    #show
    
    cv2.waitKey(0)
    
