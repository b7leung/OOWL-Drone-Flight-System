# import the necessary packages
import numpy as np
import argparse
import cv2
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())
 
# load the image
image = cv2.imread(args["image"])
# define the list of boundaries
boundaries = [
	([240, 0, 0], [255, 50, 50])
]

# loop over the boundaries
for (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")
 
	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(image, lower, upper)
	output = cv2.bitwise_and(image, image, mask = mask)
        im2, contours,hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        


        cnt=contours[0][:][:]
        
        
        mx=np.amax(cnt,0)
        mn=np.amin(cnt,0)
        
        
        print(mx)
        print(mn)

        maxx=mx[0][0]
        maxy=mx[0][1]
        minx=mn[0][0]
        miny=mn[0][1]
        #print(minx)
       #
       #print(miny)
        print(maxx)

        print(maxy)

        cnt=np.array([[[minx,miny],[minx,maxy]],[[minx,miny],[maxx,miny]],
        [[maxx,miny],[maxx,maxy]],[[maxx,maxy],[minx,maxy]]])
        
        cv2.drawContours(image, cnt, -1, (0,255,0), 3,4)	# show the images
	cv2.imshow("images", np.hstack([image, output]))
       	cv2.imwrite('index.png',output)
	cv2.waitKey(0)
