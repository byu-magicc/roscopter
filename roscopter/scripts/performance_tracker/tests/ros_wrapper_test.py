#!/usr/bin/env python

import unittest
import sys
sys.path.append('/home/matthew/ws_rtkflight/src/roscopter/roscopter/scripts/performance_tracker')

from nav_msgs.msg import Odometry
from performance_ros import PerformanceROS

class RosWrapperTest(unittest.TestCase):

    def setUp(self):
        riseTimeCutoffPercentage = 90.0
        settleTimeEnvelopePercentage = 10.0
        self.msg = Odometry()
        self.performance_ros = PerformanceROS()

    def test_odomCallback_initial_single_north_command(self):

        self.msg.pose.pose.position.x = 0.0
        self.msg.pose.pose.position.y = 0.0
        self.msg.pose.pose.position.z = 0.0

        self.msg.pose.pose.orientation.w = 1.0
        self.msg.pose.pose.orientation.x = 0.0
        self.msg.pose.pose.orientation.y = 0.0
        self.msg.pose.pose.orientation.z = 0.0

        self.msg.header.stamp.secs = 3.0
        self.msg.header.stamp.nsecs = 0.0

        self.performance_ros.performance.north.prev_hl = 0.0
        self.performance_ros.performance.east.prev_hlc = 0.0
        self.performance_ros.performance.down.prev_hlc = 0.0
        self.performance_ros.performance.yaw.prev_hlc = 0.0

        self.performance_ros.performance.north.high_level_command = 5.0
        self.performance_ros.performance.east.high_level_command = 0.0
        self.performance_ros.performance.down.high_level_command = 0.0
        self.performance_ros.performance.yaw.high_level_command = 0.0

        self.performance_ros.time.receivedCommandTime = 1.0

        self.performance_ros.performance.update_command_for_all_states()
        self.performance_ros.odomCallback(self.msg)

        self.assertEqual(self.performance_ros.performance.north.rise_time,-1.0)
        self.assertEqual(self.performance_ros.performance.east.rise_time, 2.0)
        self.assertEqual(self.performance_ros.performance.down.rise_time, 2.0)

        self.assertEqual(self.performance_ros.performance.north.settle_time,-1.0)
        self.assertEqual(self.performance_ros.performance.east.settle_time, 2.0)
        self.assertEqual(self.performance_ros.performance.down.settle_time, 2.0)

        self.assertEqual(self.performance_ros.performance.north.percent_overshoot,0.0)
        self.assertEqual(self.performance_ros.performance.east.percent_overshoot, 0.0)
        self.assertEqual(self.performance_ros.performance.down.percent_overshoot, 0.0)

    def test_odomCallback_initial_complex(self):

        self.msg.pose.pose.position.x = 1.0
        self.msg.pose.pose.position.y = -3.0
        self.msg.pose.pose.position.z = -2.0

        #about 60 degres yaw
        self.msg.pose.pose.orientation.w = 0.866
        self.msg.pose.pose.orientation.x = 0.0
        self.msg.pose.pose.orientation.y = 0.0
        self.msg.pose.pose.orientation.z = 0.5

        self.msg.header.stamp.secs = 3.0
        self.msg.header.stamp.nsecs = 0.0

        self.performance_ros.performance.north.prev_hl = -1.0
        self.performance_ros.performance.east.prev_hlc = 1.0
        self.performance_ros.performance.down.prev_hlc = -5.0
        self.performance_ros.performance.yaw.prev_hlc = 70.0

        self.performance_ros.performance.north.high_level_command = 0.95
        self.performance_ros.performance.east.high_level_command = -10.0
        self.performance_ros.performance.down.high_level_command = -2.1
        self.performance_ros.performance.yaw.high_level_command = 50.0

        self.performance_ros.time.receivedCommandTime = 1.0

        self.performance_ros.performance.update_command_for_all_states()
        self.performance_ros.odomCallback(self.msg)

        self.assertEqual(self.performance_ros.performance.north.rise_time,2.0)
        self.assertEqual(self.performance_ros.performance.east.rise_time, -1.0)
        self.assertEqual(self.performance_ros.performance.down.rise_time, 2.0)

        self.assertEqual(self.performance_ros.performance.north.settle_time,2.0)
        self.assertEqual(self.performance_ros.performance.east.settle_time, -1.0)
        self.assertEqual(self.performance_ros.performance.down.settle_time, 2.0)

        self.assertAlmostEqual(self.performance_ros.performance.north.percent_overshoot, 5.2631, 2)
        self.assertEqual(self.performance_ros.performance.east.percent_overshoot, 0.0)
        self.assertAlmostEqual(self.performance_ros.performance.down.percent_overshoot, 3.448, 2)


if __name__ == '__main__':
    unittest.main()