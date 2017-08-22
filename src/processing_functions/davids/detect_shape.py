import numpy as np
import cv2 
import sys

imagePath = sys.argv[1]

image = cv2.imread(imagePath)


image= cv2.resize(image,(0,0), fx = 0.3,fy = 0.3)
#gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
#blurred = cv2.GaussianBlur(gray, (5,5),0)
#90 40 3 239 250 190
boundaries = [([90, 50, 1], [239, 255, 240])]
for (lower,upper) in boundaries:
    lower = np.array(lower,dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    segmented = cv2.inRange(image,lower,upper)
    output = cv2.bitwise_and(image, image, mask = segmented)
    
contours = cv2.findContours(segmented.copy() , cv2.RETR_EXTERNAL,
cv2.CHAIN_APPROX_SIMPLE)
contours = contours[1]

#print(str(contours))
for c in contours:
    dontPrint = False
    perim = cv2.arcLength(c,True)
    if perim < 5:
        print("too small")

    approx = cv2.approxPolyDP(c, 0.005 * perim,True)
    
    if len(approx) < 3:
        print("error")
        dontPrint = True
        
    if len(approx) == 3:
        values = 0
        M = cv2.moments(approx)
        if(M["m00"] != 0):
            cx = int(M["m10"] / M["m00"])
            cy= int(M["m01"] / M["m00"])
            
            center = (cx,cy)
            i=0
            for arg in c:
                point = arg[0]
                dist = np.absolute(point - center)
                values += np.sqrt(np.inner(dist,dist))
                i +=1
            radius=int(values/i)
            print(radius)
            if(radius < 10):
                dontPrint = True

            
        print("triangle")
    if len(approx) == 4:
        print("rectangle")
        values = 0
        M = cv2.moments(approx)
        if(M["m00"] != 0):
            cx = int(M["m10"] / M["m00"])
            cy= int(M["m01"] / M["m00"])
        
            center = (cx,cy)
            i=0
            for arg in c:
                point = arg[0]
                dist = np.absolute(point - center)
                values += np.sqrt(np.inner(dist,dist))
                i +=1
            radius=int(values/i)
            print(radius)
            if(radius < 10):
                dontPrint = True
        else:
            dontPrint = True

    elif len(approx) > 4:
        values = 0
        M = cv2.moments(approx)
        if(M["m00"] != 0):

            cx = int(M["m10"] / M["m00"])
            cy= int(M["m01"] / M["m00"])
            center = (cx,cy)
            i=0
        
            for arg in c:
                point = arg[0]
                dist = np.absolute(point - center)
                values += np.sqrt(np.inner(dist,dist))
                i +=1
            radius=int(values/i)
            print(radius*2)
            print(radius)
            if(radius < 15):
                dontPrint = True
                print("unidentified")
            else: 
                print("circle")
            cv2.circle(output,(cx,cy),1,(255,255,255),-1)
    
        
    if(not dontPrint):
        print("drawing shape" )
        cv2.drawContours(output,[approx],-1,(0,255,0),2)

cv2.imshow("image",output)
cv2.waitKey(0)
