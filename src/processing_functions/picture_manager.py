
import rospy
import datetime
import cv2
from pathlib2 import Path

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
        
        path = self.savePath + imageName + ".png"
        pathFile = Path(path)
        index = 1
        while pathFile.is_file():
            degIndex = path.find("_ Picture")
            path = path[:degIndex] + "_" + str(index) + path[degIndex:]
            rospy.logwarn(path)
            pathFile = Path(path)
            index += 1

        cv2.imwrite( path, image)

        return imageName, path 
