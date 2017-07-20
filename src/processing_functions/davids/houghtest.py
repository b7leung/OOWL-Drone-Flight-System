import cv2
import numpy as np

img = cv2.imread('index.png')
new=cv2.imread('index.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)
cv2.imwrite('test.jpg',edges)

#print(len(edges))
print(len(edges[1]))
thresh=1000
lines = cv2.HoughLines(edges,1,np.pi/360,thresh)
print(type(lines))
x=None
print("x",x)
print(type(x))
while(type(lines) == type(None) and thresh>0):
    lines = cv2.HoughLines(edges,1,np.pi/360,thresh)
    thresh -= 1
    print("entered")
L=np.matrix(lines).mean(0)
print(lines)
size=len(lines)
row=L[0,0]
degrees=L[0,1]
for i in range(size):
    for rho,theta in lines[i]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        
        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
        print("row",rho)
        print("degrees",(theta*180)/np.pi)
        
cv2.imwrite('houghlines3.jpg',img)

print(-1*((degrees*180)/np.pi)+90)
print(row)
a = np.cos(degrees)
b = np.sin(degrees)
x0 = a*row
y0 = b*row
x1 = int(x0 + 1000*(-b))
y1 = int(y0 + 1000*(a))
x2 = int(x0 - 1000*(-b))
y2 = int(y0 - 1000*(a))
        
cv2.line(new,(x1,y1),(x2,y2),(0,0,255),2)
cv2.imwrite('houghaverage.jpg',new)
cv2.imshow("hough",new)
cv2.waitKey(0)
