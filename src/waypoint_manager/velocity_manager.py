#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from rosflight_msgs.msg import Command
from nav_msgs.msg import Odometry
from ros_copter.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
import yaml
import tf
import numpy as np
import os
import math
import sys

class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = [[0,0,-10]]
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')


        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 5)
        self.cyclical_path = rospy.get_param('~cycle', True)

        self.prev_time = rospy.Time.now()

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)

        self.current_waypoint_index = 0

        command_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])

        command_msg.x = current_waypoint[0]
        command_msg.y = current_waypoint[1]
        command_msg.F = current_waypoint[2]
        if len(current_waypoint) > 3:
            command_msg.z = current_waypoint[3]
        else:
            next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
            delta = next_point - current_waypoint
            command_msg.z = math.atan2(delta[1], delta[0])
        command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(command_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(req):
        print("addwaypoints")

    def removeWaypointCallback(req):
        print("remove Waypoints")

    def setWaypointsFromFile(req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):
        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])

        error = np.linalg.norm(current_position - current_waypoint[0:3])

        if error < self.threshold:
            # Go into stabilize mode and quite node
            xvel = 0
            yvel = 0
            yawrate = 0
            altitude = self.waypoint_list[0][2]

            command_msg = Command()
            command_msg.x = xvel
            command_msg.y = yvel
            command_msg.z = yawrate
            command_msg.F = altitude

            command_msg.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
            self.waypoint_pub_.publish(command_msg)
            self.xhat_sub_.unregister()
            self.waypoint_pub_.unregister()

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
