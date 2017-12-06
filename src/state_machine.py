#!/usr/bin/env python

import rospy

# a helper class that cycles through different specified states,
# indicating what the current state should be
class StateMachine(object):
    
    def __init__(self):

        self.states = None
        
    # Defines the state machine. it will go through 3 "phases": 
    #
    # 1. an initalize phase, for one cycle
    # 2. an algorithm phase, for a specified # of cycles
    # if the # = -1, then it represents an alg cycle of infinite length
    # 3. an ending phase, for one cycle
    #
    # Then, the machine is considered "finished".
    #
    # "Instructions" are an array of tuples. Each tuple represents a phase's instruction set and duration
    # in the format (directive, stateduration, errordirective, method). directive must subclass AbstractDroneDirective
    # method is optional, and can be used to specifically call some metohd in the directive
    # errordirectives are optional, and can specify a directive to switch to when the main directive has "errored".
    # It must also subclass AbstractDroneDirective

    def DefineMachine(self, initalizeInstructions, algorithmInstructions, algCycles, endingInstructions, receiver):
        
        # combine phases into a large array. First elem in the tuple is the instruction set for the phase;
        # second elem is the number of cycles the phase should have.
        self.stateMachineDef = [ (initalizeInstructions, 1), (algorithmInstructions, algCycles),
        (endingInstructions, 1) ]

        self.receiver = receiver

        # keeps track of how many cycles has been run in the current phase
        self.phaseCycles = 0
        # keeps track of what phase we're in (init is 0, alg is 1, end is 2)
        self.currPhase = 0
        # within some phase, keeps track of what state we're in
        self.currPhaseIndex = 0
        # for some state, keeps track of how long that state has been finished 
        self.stateFinishedCounter = 0
        
        # setting up error function
        #self.errorInstructions = errorInstructions
        self.errorFlag = False
        self.errorDuration = 0
        self.errorMaxDuration = 400
        self.lastLocation = (None, None)
        self.errorCount = 0

        self.MachineFinished = False
        self.moveTime = 0
        self.waitTime = 0


    # Returns update on what the drone should do next according to the current state
    def GetUpdate(self, image, navdata):
        
        if not self.MachineFinished:

            # edge case: when the machine has run through all its phases, it's finished
            if self.currPhase >= len( self.stateMachineDef ):
                self.MachineFinished = True
                return (0,0,0,0), image, 0, 0

            # edge case: if the current phase is "None", just go to the next phase
            if self.stateMachineDef[self.currPhase][0] == None:
                self.currPhase += 1
                return (0,0,0,0), image, 0, 0

            # edge case: if we want to just call one method
            if len(self.stateMachineDef[self.currPhase][0][self.currPhaseIndex]) == 4:
                currState = self.stateMachineDef[self.currPhase][0][self.currPhaseIndex]
                currStateDuration = currState[1]
                status, droneInstructions, _, coordinate, self.moveTime, self.waitTime, newCenter  = getattr(currState[0], currState[3])(image, navdata)

            # if the error flag was set, use it as the current state instead of the normal one
            elif self.errorFlag:
                # to do: none edge case
                currState = self.stateMachineDef[self.currPhase][0][self.currPhaseIndex][2][0]
                currStateDuration = self.stateMachineDef[self.currPhase][0][self.currPhaseIndex][2][1]
                self.errorDuration += 1
                status, droneInstructions, image, coordinate, self.moveTime, self.waitTime, newCenter = currState.RetrieveNextInstruction(image,(navdata,self.lastLocation))
                # if it's been over the specified limit without success, abort
                if self.errorDuration > self.errorMaxDuration or status == -1:
                    rospy.logwarn("********** ABORTING -- MACHINE FAILED **********") 
                    self.MachineFinished = True
            else:
                
                currState = self.stateMachineDef[self.currPhase][0][self.currPhaseIndex][0]
                currStateDuration = self.stateMachineDef[self.currPhase][0][self.currPhaseIndex][1]
                status, droneInstructions, image, coordinate, self.moveTime, self.waitTime, newCenter = currState.RetrieveNextInstruction(image,navdata)

            # if the current state is finished, increment counter
            if status == 1:

                self.stateFinishedCounter += 1
                self.lastLocation = coordinate

                # if the state has hit the specified duration to be considered "finished"
                # move on to the next state in the phase
                if self.stateFinishedCounter >= currStateDuration:
                    
                    # if the current directive has a Finished() method, call it to perform any clean-up work
                    # finished() returns either a (x,y) describing the final center of that state, or None if not applicable.
                    if ( hasattr( currState, 'Finished' ) and callable (currState.Finished ) and 
                    self.stateMachineDef[self.currPhase][1] != -1 ):
                        lastCenter = currState.Finished()
                        if lastCenter != None:
                            self.receiver.UpdateRecordedCenter(lastCenter)
                    
                    # update center if directive calls for it
                    if newCenter != None:
                        rospy.logwarn("setting new center")
                        self.receiver.SetCenter(newCenter)

                    self.stateFinishedCounter = 0
                    droneInstructions = (0,0,0,0)

                    # if error flag was on, just turn it off and go back the state before error occurred
                    if self.errorFlag:
                        rospy.logwarn("Error state over; returning")
                        oldState = self.stateMachineDef[self.currPhase][0][self.currPhaseIndex][0]
                        # if the current directive has an OnErrorReturn() method, call it to perform any clean-up work after returning
                        if ( hasattr( oldState, 'OnErrorReturn' ) and callable (oldState.OnErrorReturn) and 
                            self.stateMachineDef[self.currPhase][1] != -1 ):
                            #debt; customized to get 1st (center). should be generic
                            oldState.OnErrorReturn(coordinate[0])
                        self.errorDuration = 0
                        self.errorFlag = False
                        if isinstance(self.lastLocation[0], tuple):
                            self.receiver.SetCenter(self.lastLocation[0])
                        else:
                            self.receiver.SetCenter(self.lastLocation)
                    else:

                        self.currPhaseIndex = (self.currPhaseIndex + 1)

                        # triggered if phase itself is finished
                        if self.currPhaseIndex >= len( self.stateMachineDef[self.currPhase][0] ):
                            self.phaseCycles += 1
                            self.currPhaseIndex = 0
                            # only go to next phase if the cycles are complete for the current phase
                            if ( self.stateMachineDef[self.currPhase][1] != -1 and
                            self.phaseCycles >= self.stateMachineDef[self.currPhase][1]) :
                                self.currPhase += 1
                                self.phaseCycles = 0
                self.errorCount = 0

            # if state is still running, reset counter and let it keep continue running
            elif status == 0:

                self.stateFinishedCounter = 0
                if coordinate != (None,None):
                    self.lastLocation = coordinate
                self.errorCount = 0

            # if status = -1; error occured.
            else:
                self.errorCount +=1 
                currStateError = self.stateMachineDef[self.currPhase][0][self.currPhaseIndex]

                if len(currStateError) == 3 and currStateError[2] != None and self.errorCount >=4:
                    self.errorFlag = True

            return droneInstructions, image, self.moveTime, self.waitTime

        else:
            
            rospy.logwarn("Machine Finished")
            return (0,0,0,0), image, 0, 0
                
    

        




