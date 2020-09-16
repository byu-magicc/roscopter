import numpy as np

class Performance():

    def __init__(self, riseTimeCutoffPercentage, settleTimeEnvelopePercentage):

        #creat object to track time
        self.time = Time()

        #create objects for each dimension
        self.north = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage, self.time)
        self.east = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage, self.time)
        self.down = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage, self.time)
        self.yaw = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage, self.time)

    def update_performance_measures_for_all_states(self):

        self.north.update_performance_measures()
        self.east.update_performance_measures()
        self.down.update_performance_measures()
        self.yaw.update_performance_measures()

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

    def __init__(self, riseTimeCutoffPercentage, settleTimeEnvelopePercentage, time):

        #TODO Make sure self.time is not a copy
        self.time = time
        self.riseTimeCutoffPercentage = riseTimeCutoffPercentage
        self.settleTimeEnvelopePercentage = settleTimeEnvelopePercentage

        #Calculated parameters
        self.riseTimeCutoff = 0.0
        self.settleTimeCutoff = 0.0
        self.command_distance = 0.0

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

    def update_performance_measures(self):

        self.error = self.high_level_command - self.state
        self.update_rise_time()
        self.update_settle_time()
        self.update_percent_overshoot()

    def update_command(self):

        self.command_distance = self.high_level_command - self.prev_hlc
        self.riseTimeCutoff = self.command_distance*self.riseTimeCutoffPercentage
        self.settleTimeCutoff = self.settleTimeEnvelopePercentage*self.command_distance

        self.rise_time_set = False
        self.settle_time_set = False

        self.prev_hlc = self.high_level_command

    def update_rise_time(self):

        if self.rise_time_set == False and abs(self.error) < self.riseTimeCutoff:
            self.rise_time = self.time.odomTime - self.time.receivedCommandTime
            self.rise_time_set = True

    def update_settle_time(self):

        if self.settle_time_set == False and abs(self.error) < self.settleTimeCutoff:
            self.settle_time = self.time.odomTime - self.time.receivedCommandTime
            self.settle_time_set = True
        if self.settle_time_set == True and abs(self.error) > self.settleTimeCutoff:
            self.settle_time = 0.0
            self.settle_time_set = False

    def update_percent_overshoot(self):

        if self.command_distance >= 0.0:
            if self.error > self.percent_overshoot:
                self.percent_overshoot = self.error
        elif self.command_distance < 0.0:
            if self.error < self.percent_overshoot:
                self.percent_overshoot = self.error
        else:
            print('Invalid error in update percent overshot')
    