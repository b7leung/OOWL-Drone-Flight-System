#!/usr/bin/env python

import rospy

# a helper class that cycles through different specified states,
# indicating what the current state should be
class StateMachine(object):
    

    def __init__(self, errorFunction = None):

        self.states = None

        if errorFunction == None:
            self.errorFunction = None
            self.errorDuration = None
        else:
            self.errorFunction = errorFunction[0]
            self.errorDuration = errorFunction[1]

        
    # ArrayOfStates is an array of tuples. Each tuple represents a state's instruction set and duration
    # in the format (stateobject, stateduration). stateobject must subclass AbstractDroneDirective
    # Machine will cycle through these states in order, starting with the first one
    def SetStates(self, ArrayOfStates):
        
        self.states = ArrayOfStates
        self.currStateIndex = 0
        self.stateFinishedCounter = 0
        
        # For the error function
        self.lastLocation = (None, None)
        self.errorCounter = 0


    # get update according to the current state
    def GetUpdate(self, image, navdata):
        
        # getting the current state's directive instruction
        stateDirective = self.states[self.currStateIndex][0]
        status, droneInstructions, image, coordinate = stateDirective.RetrieveNextInstruction(image,navdata)

        # if the current state is finished, increment counter
        if status == 1:

            self.stateFinishedCounter += 1
            self.lastLocation = coordinate
            self.errorCounter = 0
            # if the state has hit the specified duration to be considered "finished"
            # move on to the next state
            if self.stateFinishedCounter >= self.states[self.currStateIndex][1]:
                self.currStateIndex = (self.currStateIndex + 1) % len(self.states)
                self.stateFinishedCounter = 0
                droneInstructions = (0,0,0,0)

        # if state is still running, reset counter
        elif status == 0:

            self.stateFinishedCounter = 0
            self.lastLocation = coordinate
            self.errorCounter = 0

        # if status = -1; error occured. calls error function, if it was passed in
        else:

            if self.errorFunction != None:

                if self.errorCounter < self.errorDuration:

                    status, droneInstructions, image, coordinate = self.errorFunction.RetrieveNextInstruction(image, self.lastLocation)
                    self.errorCounter += 1
                    
                    if status == 1:
                        rospy.logwarn("The drone has returned to platform")
                    elif status == 0:
                        rospy.logwarn("Trying to return")
                    else:
                        rospy.logwarn("Nothing to return to")

                else:

                    rospy.logwarn("FUBAR -- state machine failed")

                
        return droneInstructions, image
    

        




