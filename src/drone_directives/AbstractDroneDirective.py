#!usr/bin/env python

from abc import ABCMeta, abstractmethod

# An abstract class that represents some algorithm computing where the drone should go next.
# It will always be given information about the drone's navdata and image, and a subclass of this 
# will perform some algorithm returning where the drone should go.
#
class AbstractDroneDirective:
    __metaclass__ = ABCMeta

    # returns the next instruction on where the drone should go
    @abstractmethod
    def RetrieveNextInstruction(self, image, navdata):
        pass
