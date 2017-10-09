import numpy as np
from scipy.misc import imread, imsave
import matplotlib.pyplot as plt

#function to rotate image by 90 degrees
def rotate90(img):
    img_res = np.transpose(img,(1,0,2))
    img_res = np.flipud(img_res)
    return img_res
# Rotate image by an angle in anticlocwise direction
# angle is assumged to be divisible by 90 degrees but may be negative
def rotate(img, ang=0):
    assert ang%90==0
    #make a copy of the image so we dont alter
    img_res = img
    #convert the angle to a positive angle between 0 and 360
    angle=ang
    if angle < 0:
        angle = ( ((-1*angle)%360)*-1)+360
    angle = angle%360
    #calculate number of 90 degree rotations
    iterations = angle/90
    for x in range(0,iterations):
        img_res = rotate90(img_res)

    return img_res

#import image here
img = imread('pepsi.jpg')

#sample call to the rotate function

im90 = rotate(img,90)
im180 = rotate(img,180)
im270 = rotate(img,270)
im360= rotate(img,360)
#plotting code below
print(im90.shape)
plt.subplot(2,2,1)
plt.imshow(im90)
plt.subplot(2,2,2)
plt.imshow(im180)
plt.subplot(2,2,3)
plt.imshow(im270)
plt.subplot(2,2,4)
plt.imshow(im360)
plt.show()

