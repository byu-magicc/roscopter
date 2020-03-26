#!/usr/bin/env python3

import numpy as np
import rospy

# from geometry_msgs import Vector3Stamped
from nav_msgs.msg import Odometry
from roscopter_msgs.msg import Command
from roscopter_msgs.msg import PoseEuler
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile


class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints')
        except KeyError:
            rospy.logfatal('[waypoint_manager] waypoints not set')
            rospy.signal_shutdown('[waypoint_manager] Parameters not set')

        # self.deg_2_rad_ = np.pi / 180.0

        self.len_waypts = len(self.waypoint_list)

        # how close does the MAV need to get before going to the next waypoint?
        self.pos_threshold = rospy.get_param('~threshold', 5)
        self.heading_threshold = rospy.get_param('~heading_threshold', 0.035)  # radians
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.print_wp_reached = rospy.get_param('~print_wp_reached', True)

        # Set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometry_callback, queue_size=5)
        # self.eul_sub_ = rospy.Subscriber('state', Vector3Stamped, self.euler_callback, queue_size=5)
        self.waypoint_cmd_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.pose_pub_ = rospy.Publisher('waypt_pose_euler', PoseEuler, queue_size=5, latch=True)

        # Wait a second before we publish the first waypoint
        rospy.sleep(2)

        # Create the initial command message
        self.cmd_msg = Command()
        self.current_waypoint_index = 0
        current_waypoint = np.array(self.waypoint_list[0])

        self.publish_command(current_waypoint)


        # Initialize member n,e,d
        self.n = None
        self.e = None
        self.d = None
        self.psi = None

        self.pose_msg = PoseEuler()
        # self.pose_msg.n = 0.0
        # self.pose_msg.e = 0.0
        # self.pose_msg.d = 0.0
        # self.pose_msg.psi = 0.0
        # self.pose_pub_.publish(self.pose_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(self, req):
        #TODO
        print("[waypoint_manager] addwaypoints (NOT IMPLEMENTED)")

    def removeWaypointCallback(self, req):
        #TODO
        print("[waypoint_manager] remove Waypoints (NOT IMPLEMENTED)")

    def setWaypointsFromFile(self, req):
        #TODO
        print("[waypoint_manager] set Waypoints from File (NOT IMPLEMENTED)")

    def odometry_callback(self, msg):
        # stop iterating over waypoints if cycle==false and last waypoint reached
        if not self.cyclical_path and self.current_waypoint_index == self.len_waypts-1:
            return

        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        self.n = msg.pose.pose.position.x
        self.e = msg.pose.pose.position.y
        self.d = msg.pose.pose.position.z
        current_position = np.array([self.n,
                                     self.e,
                                     self.d])

        # orientation in quaternion form
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        # yaw from quaternion
        self.psi = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

        position_error = np.linalg.norm(current_waypoint[0:3] - current_position)
        heading_error = self.wrap(current_waypoint[3] - y)

        if position_error < self.pos_threshold and heading_error < self.heading_threshold:
            idx = self.current_waypoint_index
            if self.print_wp_reached:
                wp_str = '[waypoint_manager] Reached waypoint {}'.format(idx+1)
                rospy.loginfo(wp_str)
            # Get new waypoint index
            self.current_waypoint_index += 1
            self.current_waypoint_index %= self.len_waypts
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

            # next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

            self.publish_command(current_waypoint)



        # publish the pose_euler estimate
        self.pose_msg.n = self.n
        self.pose_msg.e = self.e
        self.pose_msg.d = self.d
        self.pose_msg.psi = self.psi
        self.pose_pub_.publish(self.pose_msg)

    # def euler_callback(self, msg):
    #     # get euler published from ekf
    #     psi_deg = msg.vector.z
    #     self.psi_rad = self.psi_deg * self.deg_2_rad_

    def publish_command(self, current_waypoint):
        self.cmd_msg.stamp = rospy.Time.now()
        self.cmd_msg.cmd1 = current_waypoint[0]
        self.cmd_msg.cmd2 = current_waypoint[1]
        self.cmd_msg.cmd3 = current_waypoint[2]

        if len(current_waypoint) > 3:
            self.cmd_msg.cmd4 = current_waypoint[3]
        else:
            next_waypoint_index = (self.current_waypoint_index + 1) % self.len_waypts
            next_waypoint = self.waypoint_list[next_waypoint_index]
            delta = next_waypoint - current_waypoint
            self.cmd_msg.cmd4 = np.arctan2(delta[1], delta[0])

        self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
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
