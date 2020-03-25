#!/usr/bin/env python

import numpy as np
import rospy

from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.msg import RelativePose
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile, ListWaypoints


class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints')
        except KeyError:
            rospy.logfatal('[waypoint_manager] waypoints not set')
            rospy.signal_shutdown('[waypoint_manager] Parameters not set')

        self.current_waypoint_index = 0

        # how close does the MAV need to get before going to the next waypoint?
        self.pos_threshold = rospy.get_param('~threshold', 5)
        self.heading_threshold = rospy.get_param('~heading_threshold', 0.035)  # radians
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.print_wp_reached = rospy.get_param('~print_wp_reached', True)

        # Set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.removeWaypointCallback)
        self.set_waypoints_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.setWaypointsFromFileCallback)
        self.list_waypoints = rospy.Service('list_waypoints', ListWaypoints, self.listWaypoints)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_cmd_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.relPose_pub_ = rospy.Publisher('relative_pose', RelativePose, queue_size=5, latch=True)

        # Wait a second before we publish the first waypoint
        rospy.sleep(2)

        # Create the initial relPose estimate message
        relativePose_msg = RelativePose()
        relativePose_msg.x = 0
        relativePose_msg.y = 0
        relativePose_msg.z = 0
        relativePose_msg.F = 0
        self.relPose_pub_.publish(relativePose_msg)

        # Create the initial command message
        self.cmd_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])
        self.publish_command(current_waypoint)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(self, req):
        # Add a waypoint to the waypoint list at the specified index.
        new_waypoint = [req.x, req.y, req.z, req.psi]
        if req.index == -1:
            index = len(self.waypoint_list)
        elif req.index > len(self.waypoint_list):
            rospy.logwarn("[waypoint_manager] Waypoint Index Out of Range")
            return
        else:
            index = req.index
        self.waypoint_list.insert(index, new_waypoint)
        if self.current_waypoint_index >= index:
            self.current_waypoint_index += 1
        rospy.loginfo("[waypoint_manager] Added New Waypoint")
        return len(self.waypoint_list)

    def removeWaypointCallback(self, req):
        # Remove a waypoint from the index
        if len(self.waypoint_list) == 1:
            rospy.logwarn("[waypoint_manager] Cannot Remove Only Waypoint")
            return len(self.waypoint_list)
        if req.index >= len(self.waypoint_list):
            rospy.logwarn("[waypoint_manager] Waypoint Index Out of Range")
            return
        del self.waypoint_list[req.index]
        removed_str = '[waypoint_manager] Waypoint {} Removed'.format(req.index+1)
        rospy.loginfo(removed_str)
        # If the current waypoint was removed, wrap then publish
        if req.index == self.current_waypoint_index:
            # stop iterating over waypoints if cycle==false and last waypoint reached
            if not self.cyclical_path and self.current_waypoint_index == len(self.waypoint_list)-1:
                self.print_wp_reached = False
                return
            else:
                # Get new waypoint index
                self.current_waypoint_index %= len(self.waypoint_list)
                current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                self.publish_command(current_waypoint)
        # Elif the current waypoint was the last, keep as last, don't publish
        elif self.current_waypoint_index >= len(self.waypoint_list):
            self.current_waypoint_index -=1
        return len(self.waypoint_list)

    def setWaypointsFromFileCallback(self, req):
        #TODO
        print("[waypoint_manager] set Waypoints from File (NOT IMPLEMENTED)")

    def listWaypoints(self, req):
        # Returns the waypoint list
        # # Print
        # print(self.waypoint_list)
        # # List as Log Info
        # waypoint_list_str = '[waypoint_manager] Waypoints: {}'.format(self.waypoint_list)
        # rospy.loginfo(waypoint_list_str)
        # List 1 at a time
        rospy.loginfo('[waypoint_manager] Waypoints:')
        i = 1
        for waypoint in self.waypoint_list:
            waypoint_str = '[waypoint_manager] {}: {}'.format(i, waypoint)
            rospy.loginfo(waypoint_str)
            i += 1
        return True

    # def clearWaypoints(self,req): #TODO

    def odometryCallback(self, msg):
        #get current position
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

        #publish the relative pose estimate
        relativePose_msg = RelativePose()
        relativePose_msg.x = current_position[0]
        relativePose_msg.y = current_position[1]
        relativePose_msg.z = y
        relativePose_msg.F = current_position[2]
        self.relPose_pub_.publish(relativePose_msg)

        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        position_error = np.linalg.norm(current_waypoint[0:3] - current_position)
        heading_error = np.abs(self.wrap(current_waypoint[3] - y))

        if position_error < self.pos_threshold and heading_error < self.heading_threshold:
            if self.print_wp_reached:
                idx = self.current_waypoint_index
                wp_str = '[waypoint_manager] Reached waypoint {}'.format(idx+1)
                rospy.loginfo(wp_str)

            # stop iterating over waypoints if cycle==false and last waypoint reached
            if not self.cyclical_path and self.current_waypoint_index == len(self.waypoint_list)-1:
                self.print_wp_reached = False
                return
            else:
                # Get new waypoint index
                self.current_waypoint_index += 1
                self.current_waypoint_index %= len(self.waypoint_list)
                current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                self.publish_command(current_waypoint)

    def publish_command(self, current_waypoint):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.x = current_waypoint[0]
        self.cmd_msg.y = current_waypoint[1]
        self.cmd_msg.F = current_waypoint[2]

        if len(current_waypoint) > 3:
            self.cmd_msg.z = current_waypoint[3]
        else:
            next_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoint_list)
            next_waypoint = self.waypoint_list[next_waypoint_index]
            delta = next_waypoint - current_waypoint
            self.cmd_msg.z = np.arctan2(delta[1], delta[0])

        self.cmd_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_cmd_pub_.publish(self.cmd_msg)

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
