#!/usr/bin/env python

import rospy
import time
import datetime


# helper class that aids in logging information to a txt file
class Logger(object):
    

    # initalies logger object with the path to log to and a short 
    # description of what it's logging
    def __init__(self, logFilePath, description):
        self.logFilePath = logFilePath
        self.logDescription = description
        self.startedLogging = False
        self.startTime = None


    # Must call Start() before attempting to log files.
    # Actually starts the timer and creates initial log file
    def Start(self):

        # creating initial log file with timestamp in the filename & log itself
        filename = (datetime.datetime.now().strftime("%I:%M:%S%p_%a-%m-%d-%Y") +
        " " + self.logDescription + " Log" + ".txt")
        if self.logFilePath[-1] != "/":
            self.logFilePath += "/"
        self.logFile = open(self.logFilePath+filename, "a")
        self.logFile.write( " ===== " + "Log created for "+ self.logDescription +" on " + 
        datetime.datetime.now().strftime("%A, %B %d %Y -- %I:%M:%S %p") + " ===== " +"\n" )

        self.startTime = time.clock()
        self.startedLogging = True
    

    # Logs a message in the file, as a new line
    def Log(self, message, timeElapsed = True):
        # in ms
        timeElapsed = (time.clock()-self.startTime)*1000
        self.logFile.write(str(timeElapsed)+ " " + message + "\n")


    # writes an ending timestamp into the log file and closes it
    def Stop(self):
        self.logFile.write( " ===== " + self.logDescription + " log finished on " + 
        datetime.datetime.now().strftime("%A, %B %d %Y -- %I:%M:%S %p") + " ===== " +"\n" )

        self.logFile.close()


