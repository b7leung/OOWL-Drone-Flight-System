#!/usr/bin/env python

import rospy

# a helper class that cycles through different specified states,
# indicating what the current state should be
class StateMachine(object):
    

    def __init__(self, errorFunction):
        self.states = None
        self.currStateInfo = None
        self.stateCounter = 0
        self.errorFunction = errorFunction[0]
        self.errorDuration = errorFunction[1]

        
    # ArrayOfStates is an array of tuples. Each tuple represents a state's instruction set and duration
    # in the format (stateobject, stateduration). stateobject must subclass AbstractDroneDirective
    def SetStates(self, ArrayOfStates):
        
        self.states = ArrayOfStates
        self.stateCounter = 0
        # first state is the first tuple in the array
        self.currStateIndex = 0
        self.lastLocation = (None, None)
        self.errorCounter = 0
    # get update according to the current state
    def GetUpdate(self, image, navdata):
        
        # getting the current state's directive instruction
        stateDirective = self.states[self.currStateIndex][0]
        status, droneInstructions, image, coordinate = stateDirective.RetrieveNextInstruction(image,navdata)

        if status == 1:

            # if the current state is finished, increment counter
            self.stateCounter += 1
            self.lastLocation = coordinate
            self.errorCounter = 0
            # if counter has hit the specified duration, move on to the next state
            if self.stateCounter >= self.states[self.currStateIndex][1]:
                self.stateCounter = 0
                # index loops back to the first state if currently at the last state
                self.currStateIndex = (self.currStateIndex + 1) % len(self.states)
                droneInstructions = (0,0,0,0)

        elif status == 0:

            # state is still running; reset counter
            self.stateCounter = 0
            self.lastLocation = coordinate
            self.errorCounter = 0
        else:
            self.errorCounter += 1
            # status = -1; error occured. call error function
            if self.errorCounter > self.errorDuration:
                status, droneInstructions, image, coordinate = self.errorFunction.RetrieveNextInstruction(image, self.lastLocation)
            else:
                rospy.logwarn("The drone was not able to return")
            if status == 0:
                rospy.logwarn("The drone is returning to platform")
            else:
                rospy.logwarn("Nothing to return to")
                
        return droneInstructions, image
    

        




