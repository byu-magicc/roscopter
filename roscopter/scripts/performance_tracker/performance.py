#Maintainer: Matthew Rydalch
#Use this to gather controller performance information.
#Performance measures of rise time, settling time, and percent overshoot are provided.
#Performance measures are provided for each dimension and yaw i.e. x, y, z, and yaw
#TODO modify the performance measures to be in the body frame rather than the inertial frame.

import numpy as np

class Performance():

    def __init__(self, riseTimeCutoffPercentage, settleTimeEnvelopePercentage):

        self.north = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage)
        self.east = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage)
        self.down = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage)
        self.yaw = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage)

    def update_performance_measures_for_all_states(self, time):

        self.north.update_performance_measures(time)
        self.east.update_performance_measures(time)
        self.down.update_performance_measures(time)
        self.yaw.update_performance_measures(time)

    def update_command_for_all_states(self):

        self.north.update_command()
        self.east.update_command()
        self.down.update_command()
        self.yaw.update_command()

class Time():

    def __init__(self):

        self.odomTime = 0.0
        self.receivedCommandTime = 0.0

class Dimension():

    def __init__(self, riseTimeCutoffPercentage, settleTimeEnvelopePercentage):

        #provided constant parameters
        self.riseTimeCutoffPercentage = riseTimeCutoffPercentage
        self.settleTimeEnvelopePercentage = settleTimeEnvelopePercentage

        #Calculated parameters
        self.riseTimeCutoff = -1
        self.settleTimeCutoff = -1
        self.command_distance = -1

        #incoming values
        self.state = 0.0
        self.odomTime = 0.0
        self.high_level_command = 0.0
        self.prev_hlc = 0.0
        self.receivedCommandTime = 0.0

        #Booleans
        self.rise_time_set = False
        self.settle_time_set = False

        #Output values
        self.rise_time = 0.0
        self.settle_time = 0.0
        self.percent_overshoot = 0.0

    def update_performance_measures(self, time):

        self.error = self.state - self.high_level_command
        self.update_rise_time(time)
        self.update_settle_time(time)
        self.update_percent_overshoot()

    def update_command(self):

        self.command_distance = self.high_level_command - self.prev_hlc
        self.riseTimeCutoff = self.command_distance*(1.0-self.riseTimeCutoffPercentage/100.0)
        self.settleTimeCutoff = self.command_distance*self.settleTimeEnvelopePercentage/100

        #reset values
        self.rise_time_set = False
        self.settle_time_set = False
        self.percent_overshoot = 0.0

        # be sure to age the high level command.  This is done in performance_ros.py

    def update_rise_time(self, time):

        if self.rise_time_set == False and abs(self.error) <= abs(self.riseTimeCutoff):
            self.rise_time = time.odomTime - time.receivedCommandTime
            self.rise_time_set = True
        elif self.rise_time_set == False:
            self.rise_time = -1

    def update_settle_time(self, time):

        if self.settle_time_set == False and abs(self.error) <= abs(self.settleTimeCutoff):
            self.settle_time = time.odomTime - time.receivedCommandTime
            self.settle_time_set = True
        elif self.settle_time_set == False:
            self.settle_time = -1
        elif self.settle_time_set == True and abs(self.error) > abs(self.settleTimeCutoff):
            self.settle_time = -1
            self.settle_time_set = False


    def update_percent_overshoot(self):

        percent_error = 0.0
        if self.command_distance != 0.0:
            percent_error = self.error/abs(self.command_distance)*100.0
        if self.command_distance >= 0.0:
            if percent_error > self.percent_overshoot:
                self.percent_overshoot = percent_error
        elif self.command_distance < 0.0:
            if percent_error < self.percent_overshoot:
                self.percent_overshoot = percent_error
        else:
            print('Invalid error in update percent overshot')    