#!/usr/bin/env python
# Python controller for landing a multirotor using relative estimates

import rospy
from math import *
import numpy as np
import time
# from dynamic_reconfigure.server import Server
#from boat_landing.cfg import LandingControllerConfig
from rosflight_msgs.msg import Command
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
# from simple_pid import PID
import tf


class VelocityCommander:

    # Init function
    def __init__(self):

        # Init Params here

        self.x_vel = 2.0
        self.y_vel = 2.0
        self.z_vel = 0.0  # yaw rate
        self.alt_command = 10.0

        self.delay_time = 5

        # Init Publishers
        self.high_lvl_commands_pub = rospy.Publisher('/leo/velocity_command', Command, queue_size=10)

        # Init the command
        self.relative_cmd = Command()

        # begin sending commands
        self.sendCommands()

    def sendCommands(self):

        time.sleep(1)

        # send just x command
        # pack up the command
        print 'sending velocity command 1'
        self.relative_cmd.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
        self.relative_cmd.x = self.x_vel
        self.relative_cmd.y = 0.0
        self.relative_cmd.z = self.z_vel
        self.relative_cmd.F = -self.alt_command

        self.high_lvl_commands_pub.publish(self.relative_cmd)

        # wait
        time.sleep(self.delay_time)

        # send just y command
        # pack up the command
        print 'sending velocity command 2'
        self.relative_cmd.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
        self.relative_cmd.x = 0.0
        self.relative_cmd.y = self.y_vel
        self.relative_cmd.z = self.z_vel
        self.relative_cmd.F = -self.alt_command

        self.high_lvl_commands_pub.publish(self.relative_cmd)

        # wait
        time.sleep(self.delay_time)

        # send x and y command
        # pack up the command
        print 'sending velocity command 3'
        self.relative_cmd.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
        self.relative_cmd.x = -self.x_vel
        self.relative_cmd.y = -self.y_vel
        self.relative_cmd.z = self.z_vel
        self.relative_cmd.F = -self.alt_command

        self.high_lvl_commands_pub.publish(self.relative_cmd)

        # wait
        time.sleep(self.delay_time)

        # tell it to stop
        # pack up the command
        print 'sending velocity command 4'
        self.relative_cmd.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
        self.relative_cmd.x = 0.0
        self.relative_cmd.y = 0.0
        self.relative_cmd.z = 0.0
        self.relative_cmd.F = -self.alt_command

        self.high_lvl_commands_pub.publish(self.relative_cmd)

        print "done"



##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':

    # Initialize Node
    rospy.init_node('velocity_commander')

    # init path_follower object
    controller = VelocityCommander()

    rospy.spin()
