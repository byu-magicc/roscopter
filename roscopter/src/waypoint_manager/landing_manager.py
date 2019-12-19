#!/usr/bin/env python

import numpy as np
import rospy
import std_msgs.msg

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from geometry_msgs.msg import Twist


class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints') #params are loaded in launch file
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')

        #initialize platform location
        self.plt_odom = np.array([0.0,
                                  0.0,
                                  0.0])

        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 5)
        self.cyclical_path = rospy.get_param('~cycle', True)

        self.prev_time = rospy.Time.now()

        # Wait a second before we publish the first waypoint
        while (rospy.Time.now() < rospy.Time(2.)):
            pass

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.plt_odom_sub_ = rospy.Subscriber('platform_odom', Odometry, self.pltOdomCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.is_landing_pub_ = rospy.Publisher('is_landing', Bool, queue_size=5, latch=True)
       
        self.current_waypoint_index = 0
        self.landing_mode = 0

        command_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])

        command_msg.header.stamp = rospy.Time.now()
        command_msg.x = current_waypoint[0]
        command_msg.y = current_waypoint[1]
        command_msg.F = current_waypoint[2]
        if len(current_waypoint) > 3:
            command_msg.z = current_waypoint[3]
        else:
            command_msg.z = 0.
        command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(command_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def odometryCallback(self, msg):
        # Get error between waypoint and current state
        current_position = np.array([msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                -msg.pose.pose.position.z])

        if self.current_waypoint_index < len(self.waypoint_list)-1:
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            error = np.linalg.norm(current_position - current_waypoint[0:3])
            if error < self.threshold:
                # Get new waypoint index
                self.current_waypoint_index += 1
                next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                command_msg = Command()
                command_msg.header.stamp = rospy.Time.now()
                command_msg.x = next_waypoint[0]
                command_msg.y = next_waypoint[1]
                command_msg.F = next_waypoint[2]
                if len(next_waypoint) > 3:
                    command_msg.z = next_waypoint[3]
                else:
                    command_msg.z = 0.
                command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
                self.waypoint_pub_.publish(command_msg)
        else:
            command_msg = Command()
            self.is_landing_pub_.publish(True)
            next_waypoint = self.plt_odom
            error = np.linalg.norm(current_position[0:2]-next_waypoint[0:2])
            if error > self.threshold:
                self.landing_mode = 1
                command_msg.F = next_waypoint[2]+5.0
            else:
                self.landing_mode = 2
                command_msg.F = next_waypoint[2]-1.0
            command_msg.header.stamp = rospy.Time.now()
            command_msg.x = next_waypoint[0]
            command_msg.y = next_waypoint[1]
            #command_msg.f is calculated in if statement
            command_msg.z = 0
            command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            self.waypoint_pub_.publish(command_msg)


    def pltOdomCallback(self, msg):
        self.plt_odom = np.array([msg.pose.pose.position.x,
                                  -msg.pose.pose.position.y,
                                  msg.pose.pose.position.z])

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
