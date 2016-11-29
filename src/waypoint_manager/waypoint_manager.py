#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from fcu_common.msg import ExtendedCommand
from nav_msgs.msg import Odometry
from ros_copter.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
import yaml
import tf
import numpy as np
<<<<<<< HEAD
import os
import math
=======
import sys
>>>>>>> master

class WaypointManager():

    def __init__(self):

        # get parameters
        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 5)
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.waypoint_filename = rospy.get_param('~waypoint_filename', '../../params/waypoints.yaml')

        self.prev_time = rospy.Time.now()

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('waypoint', ExtendedCommand, queue_size=5, latch=True)

        # Start Up Waypoint List
        try:
            file = open(self.waypoint_filename, 'r')
            self.waypoint_list = yaml.safe_load(file)
        except:
            print "error opening " + self.waypoint_filename

        if not self.waypoint_list:
            print "UNABLE TO LOAD WAYPOINTS FILE ***************************"
        else:
            print self.waypoint_list

        self.current_waypoint_index = 0

        command_msg = ExtendedCommand()
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
        command_msg.mode = ExtendedCommand.MODE_XPOS_YPOS_YAW_ALTITUDE
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

        error = np.linalg.norm(current_position - current_waypoint)

        if error < self.threshold:
            # Get new waypoint index
            self.current_waypoint_index += 1
            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)
            else:
                if self.current_waypoint_index > len(self.waypoint_list):
                    self.current_waypoint_index -=1
            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            command_msg = ExtendedCommand()
            command_msg.x = next_waypoint[0]
            command_msg.y = next_waypoint[1]
            command_msg.F = next_waypoint[2]
            if len(current_waypoint) > 3:
                command_msg.z = current_waypoint[3]
            else:
                next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
                delta = next_point - current_waypoint
                command_msg.z = math.atan2(delta[1], delta[0])
            command_msg.mode = ExtendedCommand.MODE_XPOS_YPOS_YAW_ALTITUDE
            self.waypoint_pub_.publish(command_msg)

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
