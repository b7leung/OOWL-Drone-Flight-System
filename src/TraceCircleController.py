#!/usr/bin/env python

import rospy
import time

from os.path import expanduser

from drone_video import DroneVideo
from drone_controller import BasicDroneController
from processing_functions.process_video import ProcessVideo
from processing_functions.process_position import DronePosition


import cv2

class TraceCircleController(DroneVideo):

    def __init__(self):


        # Call DroneVideo's constructor so we are subscribed to video feed 
        # will be converted to CVImage and edited by EditVideo() before shown.
        super(TraceCircleController,self).__init__()
        
        self.process = ProcessVideo()
        #self.position = DronePosition()
        self.controller = BasicDroneController()
        self.startControl = False
        self.startTime = time.clock()
        startTimer = time.clock()

        # setting up log file (also clears the previous one)
        self.logFilePath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/output.txt"
        open(self.logFilePath, "w").close()
        self.logFile=open( self.logFilePath,"a")
        

    # define any keys to listen to here
    def KeyListener(self):
        
        #waits for a keypress indefinitly
        key=cv2.waitKey(1) & 0xFF

        if key == ord('i'):
            orange_image=self.process.DetectColor(self.cv_image,'orange')
            cx,cy=self.process.CenterofMass(orange_image)
            xspeed,yspeed=self.process.ApproximateSpeed(orange_image,cx,cy)
            
            # logging information; timeElapsed is in milliseconds
            timeElapsed = (time.clock()-self.startTime)*1000
            currentDroneInfo = "time: " +str(timeElapsed)+ " cx: " + str(cx) + " cy: " + str(cy) + " xspeed: " + str(xspeed) + " yspeed: " + str(yspeed) + "\n"
            self.logFile.write(currentDroneInfo)
            rospy.logwarn(currentDroneInfo)

            self.MoveFixedTime(xspeed,yspeed,0.2,0.5)
            
            #self.controller.SetCommand(roll=xspeed,pitch=yspeed)
            #self.position.DroneHover(xspeed,yspeed)
            self.cv_image=orange_image

        elif key == ord('p'):
            if self.startControl:
                self.startControl= False
            else:
                self.startControl = True





    # overriding superclass's EditVideo method
    # can change self.cv_image here, and changes will be reflected on the video
    def EditVideo(self):

        self.cv_image=self.process.DetectColor(self.cv_image,'orange')

        if self.startControl:
            self.goForwardIfOrange()
        
        #self.cv_image = self.process.ShowLine(self.cv_image)



    # if 0.2 % of what the drone sees is orange, then it will go forward
    # orange corresponds to a hueMin of 0 and a hueMax of 50
    def goForwardIfOrange(self):
        orangeVisible =self.process.isHueDominant(self.cv_image, 0, 50, 0.2); 
        if orangeVisible:
            rospy.logwarn("go forward")
            self.controller.SetCommand(pitch = 0.03)
        else:
            rospy.logwarn("stop")
            self.controller.SetCommand(pitch = 0)


if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.spin()
    self.logFile.close()

    

