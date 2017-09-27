
import rospy
import datetime
import cv2

class PictureManager(object):

    def __init__(self, savePath):

        if savePath[-1] != "/":
            savePath+= "/"
        self.savePath = savePath

        
    # given an image and optionally a different path than specified during object 
    # creation, will save a timestamped .PNG image to that location
    # returns the name of the picture taken
    def Capture(self, image, imagePath = None, imageName = None):

        if imagePath is None:
            imagePath = self.savePath

        if imageName is None:
            imageName = datetime.datetime.now().isoformat()

        cv2.imwrite(self.savePath + imageName + ".png", image)

        return imageName 
