#!/usr/bin/env python

import numpy as np
import rospy
import std_msgs.msg

import std_msgs.msg
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile


class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints')
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')


        # how close does the MAV need to get before going to the next waypoint?
        self.pos_threshold = rospy.get_param('~threshold', 5)
        self.heading_threshold = rospy.get_param('~heading_threshold', 0.035)  # radians
        self.cyclical_path = rospy.get_param('~cycle', True)

        self.prev_time = rospy.Time.now()

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Wait a second before we publish the first waypoint
        while (rospy.Time.now() < rospy.Time(2.)):
            pass

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)

        self.current_waypoint_index = 0

        self.cmd_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.x = current_waypoint[0]
        self.cmd_msg.y = current_waypoint[1]
        self.cmd_msg.F = current_waypoint[2]
        if len(current_waypoint) > 3:
            self.cmd_msg.z = current_waypoint[3]
        else:
            next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
            delta = next_point - current_waypoint
            self.cmd_msg.z = np.arctan2(delta[1], delta[0])
        self.cmd_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(self.cmd_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(self, req):
        print("addwaypoints")

    def removeWaypointCallback(self, req):
        print("remove Waypoints")

    def setWaypointsFromFile(self, req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):
        if not self.cyclical_path and self.current_waypoint_index == len(self.waypoint_list)-1:
            return
        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
                                     
        # orientation in quaternion form
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        # yaw from quaternion
        y = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

        position_error = np.linalg.norm(current_waypoint[0:3] - current_position)
        heading_error = self.wrap(current_waypoint[3] - y)

        if position_error < self.pos_threshold and heading_error < self.heading_threshold:
            self.cmd_msg.header.stamp = rospy.Time.now()
            # Get new waypoint index
            self.current_waypoint_index += 1
            self.current_waypoint_index %= len(self.waypoint_list)
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

            self.cmd_msg.x = current_waypoint[0]
            self.cmd_msg.y = current_waypoint[1]
            self.cmd_msg.F = current_waypoint[2]
            
            if len(current_waypoint) > 3:
                self.cmd_msg.z = current_waypoint[3]
            else:
                next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
                delta = next_point - current_waypoint
                self.cmd_msg.z = np.atan2(delta[1], delta[0])

            self.cmd_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            self.waypoint_pub_.publish(self.cmd_msg)
    
    
    def wrap(self, angle):
        angle -= 2*np.pi * np.floor((angle + np.pi) / (2*np.pi))
        return angle

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
