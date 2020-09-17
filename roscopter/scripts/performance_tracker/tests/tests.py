#!/usr/bin/env python

import unittest
import sys
sys.path.append('/home/matthew/ws_rtkflight/src/roscopter/roscopter/scripts/performance_tracker')
from performance import Dimension
from performance import Time
from performance import Performance

class Test(unittest.TestCase):

    def setUp(self):
        riseTimeCutoffPercentage = 90.0
        settleTimeEnvelopePercentage = 10.0
        self.dimension = Dimension(riseTimeCutoffPercentage, settleTimeEnvelopePercentage)
        self.time = Time()

    def test_update_rise_time_increasing_state(self):
        self.dimension.rise_time = 1.0
        self.dimension.rise_time_set = False
        self.dimension.error = 1.0
        self.dimension.riseTimeCutoff = 2.0
        self.time.odomTime = 3.0
        self.time.receivedCommandTime = 1.2
        
        self.dimension.update_rise_time(self.time)

        self.assertEqual(self.dimension.rise_time_set, True)
        self.assertEqual(self.dimension.rise_time, 1.8)

    def test_update_rise_time_decreasing_state(self):
        self.dimension.rise_time = 1.0
        self.dimension.rise_time_set = False
        self.dimension.error = -1.7
        self.dimension.riseTimeCutoff = 2.0
        self.time.odomTime = 3.7
        self.time.receivedCommandTime = 1.2

        self.dimension.update_rise_time(self.time)

        self.assertEqual(self.dimension.rise_time_set, True)
        self.assertEqual(self.dimension.rise_time, 2.5)

    def test_update_rise_time_not_risen(self):
        self.dimension.rise_time = -1
        self.dimension.rise_time_set = False
        self.dimension.error = -3.2
        self.dimension.riseTimeCutoff = -2.0
        self.time.odomTime = 3.7
        self.time.receivedCommandTime = 1.2

        self.dimension.update_rise_time(self.time)

        self.assertEqual(self.dimension.rise_time_set, False)
        self.assertEqual(self.dimension.rise_time, -1)
        
    def test_update_settle_time_settle(self):
        self.dimension.settle_time = -1
        self.dimension.settle_time_set = False
        self.dimension.error = 0.7
        self.dimension.settleTimeCutoff = 0.8
        self.time.odomTime = 5.4
        self.time.receivedCommandTime = 1.4

        self.dimension.update_settle_time(self.time)

        self.assertEqual(self.dimension.settle_time, 4.0)
        self.assertEqual(self.dimension.settle_time_set, True)

    def test_update_settle_time_not_settled(self):
        self.dimension.settle_time = -1
        self.dimension.settle_time_set = False
        self.dimension.error = 1.1
        self.dimension.settleTimeCutoff = 0.8
        self.time.odomTime = 5.4
        self.time.receivedCommandTime = 1.4

        self.dimension.update_settle_time(self.time)

        self.assertEqual(self.dimension.settle_time, -1)
        self.assertEqual(self.dimension.settle_time_set, False)

    def test_update_settle_time_already_settled(self):
        self.dimension.settle_time = 1.2
        self.dimension.settle_time_set = True
        self.dimension.error = 0.2
        self.dimension.settleTimeCutoff = 0.8
        self.time.odomTime = 5.4
        self.time.receivedCommandTime = 1.4

        self.dimension.update_settle_time(self.time)

        self.assertEqual(self.dimension.settle_time, 1.2)
        self.assertEqual(self.dimension.settle_time_set, True)

    def test_update_settle_time_no_longer_settle(self):
        self.dimension.settle_time = 1.2
        self.dimension.settle_time_set = True
        self.dimension.error = -1.3
        self.dimension.settleTimeCutoff = 0.8
        self.time.odomTime = 5.4
        self.time.receivedCommandTime = 1.4

        self.dimension.update_settle_time(self.time)

        self.assertEqual(self.dimension.settle_time, -1)
        self.assertEqual(self.dimension.settle_time_set, False)

    def test_update_overshoot_positive_not_risen(self):
        self.dimension.percent_overshoot = 0.0
        self.dimension.command_distance = 10.0
        self.dimension.error = -2.0

        self.dimension.update_percent_overshoot()

        self.assertEqual(self.dimension.percent_overshoot, 0.0)

    def test_update_overshoot_negative_not_risen(self):
        self.dimension.percent_overshoot = 0.0
        self.dimension.command_distance = -10.0
        self.dimension.error = 8.0

        self.dimension.update_percent_overshoot()

        self.assertEqual(self.dimension.percent_overshoot, 0.0)

    def test_update_overshoot_positive_overshot(self):
        self.dimension.percent_overshoot = 2.0
        self.dimension.command_distance = 10.0
        self.dimension.error = 1.0

        self.dimension.update_percent_overshoot()

        self.assertEqual(self.dimension.percent_overshoot, 10.0)

    def test_update_overshoot_negative_overshot(self):
        self.dimension.percent_overshoot = 3.0
        self.dimension.command_distance = -10.0
        self.dimension.error = -0.8

        self.dimension.update_percent_overshoot()

        self.assertEqual(self.dimension.percent_overshoot, -8.0)

    def test_update_command(self):
        self.dimension.high_level_command = 3.1
        self.dimension.prev_hlc = -2.0

        self.dimension.update_command()

        self.assertEqual(self.dimension.command_distance, 5.1)
        self.assertEqual(self.dimension.prev_hlc, -2.0)

if __name__ == '__main__':
    unittest.main()