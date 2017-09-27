#!usr/bin/env python

import rospy
import time
import datetime
from AbstractDroneDirective import *
from processing_functions.picture_manager import PictureManager

# Will save the image passed in
class CapturePhotoDirective(AbstractDroneDirective):

    # sets up this directive
    # picture path: location to save this photo
    # picturesToTake: the # of pictures to take until this directive is considred done
    # pause: time to wait before taking the next picture, in seconds
    def __init__(self, picturePath, picturesToTake = 1, pause = 0.5, objectName = "", angles = 0):

        self.pictureManager = PictureManager(picturePath)
        self.picturesToTake = picturesToTake
        self.pause = pause
        self.objectName = objectName
        # describes the nth time this class has been called in the state machine
        self.captureRound = 1
        self.picturesTaken = 1
        self.lastTaken = time.clock()
        self.shownInitMessage = False
        self.angles = angles
    

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if the algorithm is still working on taking pictures
    #   1 if algorithm is finished and the specified # of pictures have been taken
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        if self.picturesTaken == 0 and not self.shownInitMessage:
            rospy.logwarn("Taking " + str(self.picturesToTake) + " pictures ...")
            self.shownInitMessage = True
        
        if self.picturesTaken <= self.picturesToTake:

            if (time.clock() - self.lastTaken) > self.pause:
                if self.angles != 0:
                    currAngle = 360/self.angles * (self.captureRound - 1)
                else:
                    currAngle = 0
                pictureName = ( str(currAngle) + " Degrees _ Picture " + str(self.picturesTaken) 
                + " _ " + str(navdata["altitude"][1]) + " mm _ " + self.objectName + " _ " + datetime.datetime.now().isoformat() )
                pictureName = self.pictureManager.Capture(image, imageName = pictureName)
                rospy.logwarn("Saved picture # " + str(self.picturesTaken) + " as " + pictureName)
                self.picturesTaken += 1
                self.lastTaken = time.clock()

            directiveStatus = 0

        else:
            rospy.logwarn("****** Successfully took " + str(self.picturesToTake) + " pictures ******")
            directiveStatus = 1

        return directiveStatus, (0, 0, 0, 0.02), image, (None, None), 0, 0
    

    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        rospy.logwarn("Resetting picture counter")
        self.picturesTaken = 1
        self.captureRound += 1
        self.shownInitMessage = False

