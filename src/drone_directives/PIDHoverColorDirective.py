#!usr/bin/env python

import rospy
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
from processing_functions.pid_controller import PIDController
from os.path import expanduser

# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class PIDHoverColorDirective(AbstractDroneDirective):
    
    # sets up this directivep
    # platformColor: color to hover over
    def __init__(self, platformColor):

        self.platformColor = platformColor 
        self.processVideo = ProcessVideo()
        self.settingsPath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/calibratersettings.txt"

        P,I,D = self.GetSettings()
        self.pid = PIDController(P, I, D)
        #self.WriteSettings(P,I,D)
    
        
    def WriteSettings(self,P,I,D):
        File  = open(self.settingsPath, 'a') 
        File.write(str(P)+" ")
        File.write(str(I)+" ")
        File.write(str(D)+"\n")
        File.close()

    def GetSettings(self):
        # read a text file as a list of lines
        # find the last line, change to a file you have
        fileHandle = open ( self.settingsPath,'r' )
        last = fileHandle.readlines()
        fileHandle.close()        
        rospy.logwarn(str(last[0]))
        last=str(last[len(last)-1]).split()
        p, i, d = [float(x) for x in (last)]
        return p, i ,d


    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone isn't on orange yet
    #   1 if algorithm is finished and drone is now on orange
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):
        
        orange_image = self.processVideo.DetectColor(image, self.platformColor)
        cx, cy = self.processVideo.CenterOfMass(orange_image)
        self.pid.UpdateDeltaTime()
        self.pid.SetPoint(orange_image)
        self.pid.UpdateError(cx,cy)
        p,i,d= self.GetSettings()
        
        self.pid.ResetPID(p,i,d)
        self.pid.SetPIDTerms()
        #rospy.logwarn("cx:"+str(cx)+"cy:"+str(cy))
        xspeed, yspeed = self.pid.GetPIDValues()
        self.pid.DrawArrow(orange_image,xspeed,yspeed)


        #rospy.logwarn(str(xspeed)+"  "+ str(yspeed))
        #self.MoveFixedTime(xspeed, yspeed, 0 ,0, 0.1, 0.01)

        # if there is orange in the screen, and the drone is in the middle, return true
        if cx != None and cy != None and xspeed == 0 and yspeed == 0:

            rospy.logwarn("PID: Done Hovering on " + self.platformColor)
            directiveStatus = 1

        elif cx == None or cy == None:
            
            directiveStatus = -1

        else:

            rospy.logwarn("PID: Trying to Hover on " + self.platformColor)
            directiveStatus = 0

        return directiveStatus, (xspeed, yspeed, 0, 0), orange_image, (cx,cy)




        

