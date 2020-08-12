#!/usr/bin/env python3

import numpy as np
import rospy
import csv

# from geometry_msgs import Vector3Stamped
from roscopter_msgs.msg import Command as roscopter_command
from rosflight_msgs.msg import Command as rosflight_command


class CommandConverter():

    def __init__(self):

        # Set Up Publishers and Subscribers
        self.rosflight_cmd_sub_ = rospy.Subscriber('rosflight_command', rosflight_command, self.commandCallback, queue_size=5)
        self.roscopter_cmd_pub_ = rospy.Publisher('roscopter_command', roscopter_command, queue_size=5, latch=True)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def commandCallback(self, msg):
        # This Function adds a waypoint to the waypoint list at the specified index.
        copter_cmd = roscopter_command()
        copter_cmd.cmd1 = msg.x
        copter_cmd.cmd2 = msg.y
        copter_cmd.cmd3 = msg.z
        copter_cmd.cmd4 = msg.F
        if msg.mode == rosflight_command.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
            copter_cmd.mode = roscopter_command.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
        else:
            copter_cmd.mode = roscopter_command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        self.roscopter_cmd_pub_.publish(copter_cmd)
        return

if __name__ == '__main__':
    rospy.init_node('command_converter', anonymous=True)
    try:
        command_converter = CommandConverter()
    except:
        rospy.ROSInterruptException
    pass