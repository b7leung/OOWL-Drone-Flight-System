#!usr/bin/env python

import rospy
import time
import datetime
from AbstractDroneDirective import *
from processing_functions.picture_manager import PictureManager

# Will save the image passed in
class CapturePhotoDirective2(AbstractDroneDirective):

    # sets up this directive
    # picture path: location to save this photo
    # picturesToTake: the # of pictures to take until this directive is considred done
    # pause: time to wait before taking the next picture, in seconds
    # objectName: name of picture to be taken (will be appended to picture filename
    # angles: Number of angles that is being used for the circle
    # objectAltitude: how high off the ground the object being taken is
    def __init__(self, picturePath, picturesToTake = 1, pause = 0.5, objectName = "", angles = 0, captureRound=1, objectAltitude = None):

        self.pictureManager = PictureManager(picturePath)
        self.picturesToTake = picturesToTake
        self.pause = pause
        self.objectName = objectName
        self.angles = angles
        self.objectAltitude = objectAltitude

        # describes the nth time this class has been called in the state machine
        self.captureRound = captureRound
        # describes the # of times pictures have been taken for this state call
        self.picturesTaken = 1
        self.lastTaken = time.clock()
        self.shownInitMessage = False
    

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
            # contents are called if it's time to take a picture
            if (time.clock() - self.lastTaken) > self.pause:
                
                # creating the filename for this particular picture
                if self.angles != 0:
                    currAngle = 360/self.angles * (self.captureRound - 1)
                else:
                    currAngle = 0
                if self.objectAltitude == None:
                    altitude = str(navdata["altitude"][1])
                else:
                    offset = 200
                    altitude = navdata["altitude"][1] + offset - self.objectAltitude
                    if altitude > 0:
                        altitude = "+" + str(altitude)
                    else:
                        altitude = str(altitude)
                pictureName = ( str(currAngle) + " Degrees _ Picture " + str(self.picturesTaken) 
                + " _ " + altitude + " mm _ " + self.objectName + " _ " + datetime.datetime.now().isoformat() )
                pictureName = self.pictureManager.Capture(image, imageName = pictureName)
                rospy.logwarn("Saved picture # " + str(self.picturesTaken) + " as " + pictureName)
                self.picturesTaken += 1
                self.lastTaken = time.clock()

            directiveStatus = 0

        else:
            rospy.logwarn("****** Successfully took " + str(self.picturesToTake) + " pictures ******")
            directiveStatus = 1

        return directiveStatus, (0, 0, 0, 0.0), image, (None, None), 0, 0
    

    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        rospy.logwarn("Resetting picture counter")
        self.picturesTaken = 1
        self.captureRound += 1
        self.shownInitMessage = False
