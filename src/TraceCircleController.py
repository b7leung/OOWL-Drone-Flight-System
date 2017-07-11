#!/usr/bin/env python

import rospy

from drone_video import DroneVideo

from processing_functions.process_video import ProcessVideo


class TraceCircleController(DroneVideo):

    def __init__(self):
        super(TraceCircleController,self).__init__()
        self.process = ProcessVideo()



if __name__=='__main__':
    
    rospy.init_node('TraceCircleController')
    controller = TraceCircleController()
    rospy.spin()
    

