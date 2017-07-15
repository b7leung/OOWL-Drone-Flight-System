#!/usr/bin/env python

import rospy

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

    # define any keys to listen to here
    def KeyListener(self):
        
        #waits for a keypress indefinitly
        key=cv2.waitKey(1) & 0xFF
        if key == ord('t'):
            rospy.logwarn('0')
            self.controller.SendTakeoff()

        elif key == ord(' '):
            rospy.logwarn('1')
            self.controller.SendLand()

        elif key == ord('w'):
            self.controller.SetCommand(pitch=0.5)

        elif key == ord('s'):
            self.controller.SetCommand(pitch=-0.5)

        elif key == ord('d'):
            self.controller.SetCommand(roll=0.5)

        elif key == ord('a'):
            self.controller.SetCommand(roll=-0.5)

        elif key == ord('h'):
            self.controller.SetCommand(roll=0, pitch=0)

        elif key == ord('i'):
            rospy.logwarn('2')
            orange_image=self.process.DetectColor(self.cv_image,'orange')
            cx,cy=self.process.CenterofMass(orange_image)
            xspeed,yspeed=self.process.ApproximateSpeed(orange_image,cx,cy)
            self.controller.SetCommand(roll=xspeed,pitch=yspeed)
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

            # if 0.2 % of what the drone sees is orange, then it will go forward
            # orange corresponds to a hueMin of 0 and a hueMax of 50
            orangeVisible =self.process.isHueDominant(self.cv_image, 0, 50, 0.2); 
            if orangeVisible:
                rospy.logwarn("go forward")
                self.controller.SetCommand(pitch = 0.02)
            else:
                rospy.logwarn("stop")
                self.controller.SetCommand(pitch = 0)

        #self.cv_image = self.process.ShowLine(self.cv_image)

if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.spin()
    

