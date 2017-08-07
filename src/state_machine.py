#!/usr/bin/env python

import rospy

# a helper class that cycles through different specified states,
# indicating what the current state should be
class StateMachine(object):
    

    def __init__(self):
        self.states = None
        self.currStateInfo = None
        self.stateCounter = 0


    # ArrayOfStates is an array of tuples. Each tuple represents a state's instruction set and duration
    # in the format (stateobject, stateduration). stateobject must subclass AbstractDroneDirective
    def SetStates(self, ArrayOfStates):
        
        self.states = ArrayOfStates
        self.stateCounter = 0
        # first state is the first tuple in the array
        self.currStateIndex = 0


    # get update according to the current state
    def GetUpdate(self, image, navdata):
        
        # getting the current state's directive instruction
        stateDirective = self.states[self.currStateIndex][0]
        status, droneInstructions, image = stateDirective.RetrieveNextInstruction(image,navdata)

        if status == 1:
            # if the current state is finished, increment counter
            self.stateCounter += 1

            # if counter has hit the specified duration, move on to the next state
            if self.stateCounter >= self.states[self.currStateIndex][1]:
                self.stateCounter = 0
                # index loops back to the first state if currently at the last state
                self.currStateIndex = (self.currStateIndex + 1) % len(self.states)

        else:
            # otherwise, reset counter
            self.stateCounter = 0

        return droneInstructions, image
    

        




