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
        self.position = DronePosition()
        self.controller = BasicDroneController()
        

    # define any keys to listen to here
    def KeyListener(self):
        
        #waits for a keypress indefinitly
        key=cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            rospy.logwarn('0')
            self.controller.SendTakeoff()

        elif key == ord('a'):
            rospy.logwarn('1')
            self.controller.SendLand()


    # overriding superclass's EditVideo method
    # can change self.cv_image here, and changes will be reflected on the video
    def EditVideo(self):
        self.cv_image = self.process.ShowLine(self.cv_image)
        pass


if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    trace = TraceCircleController()
    rospy.spin()
    

