#!/usr/bin/env python

import numpy as np
import rospy
import std_msgs.msg

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


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
        self.plt_prev_odom = np.array([0.0,
                                       0.0,
                                       0.0])
        self.landing_safety_height = rospy.get_param('~landing_safety_height', 2.0)
        self.land = 0.0

        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 5)
        self.cyclical_path = rospy.get_param('~cycle', False)

        self.prev_time = rospy.Time.now()
        self.plt_prev_time = 0.0

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Wait a second before we publish the first waypoint
        while (rospy.Time.now() < rospy.Time(2.)):
            pass

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.plt_odom_sub_ = rospy.Subscriber('platform_odom', Odometry, self.pltOdomCallback, queue_size=5)
        self.plt_pose_sub_ = rospy.Subscriber('platform_pose', PoseStamped, self.pltPoseCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.is_landing_pub_ = rospy.Publisher('is_landing', Bool, queue_size=5, latch=True)
        self.platform_virtual_odom_pub_ = rospy.Publisher('platform_virtual_odometry', Odometry, queue_size=5, latch=True)
        
        self.is_landing = rospy.get_param('~is_landing', False)
        self.current_waypoint_index = 0
        self.landing_mode = 0 #landing mode 0, try to hover over the target

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


    #TODO: Need to set up these services
    def addWaypointCallback(req):
        print("addwaypoints")

    def removeWaypointCallback(req):
        print("remove Waypoints")

    def setWaypointsFromFile(req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):
                # Get error between waypoint and current state
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])

        if self.current_waypoint_index < len(self.waypoint_list):
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            error = np.linalg.norm(current_position - current_waypoint[0:3])

            if error < self.threshold:
                # Get new waypoint index
                self.current_waypoint_index += 1
                if self.current_waypoint_index == len(self.waypoint_list):
                    return

                if self.cyclical_path:
                    self.current_waypoint_index %= len(self.waypoint_list)                
                else:

                    if not self.is_landing and self.current_waypoint_index > len(self.waypoint_list):
                        self.current_waypoint_index -=1

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
            self.is_landing_pub_.publish(True)
            if self.is_landing and self.current_waypoint_index == len(self.waypoint_list)+1:
                waypoint = self.plt_odom + np.array([0.0, 0.0, self.land])
            else:# self.is_landing and self.current_waypoint_index == len(self.waypoint_list)+1:
                waypoint = self.plt_odom+np.array([0.0, 0.0, self.landing_safety_height])

            error = np.linalg.norm(current_position - waypoint[0:3])
            if error < self.threshold:
                # Get new waypoint index
                self.current_waypoint_index += 1

                if self.current_waypoint_index > len(self.waypoint_list)+1:
                    self.current_waypoint_index -=1
                    self.land = -0.5

            command_msg = Command()
            command_msg.header.stamp = rospy.Time.now()
            command_msg.x = waypoint[0]
            command_msg.y = waypoint[1]
            command_msg.F = waypoint[2]

            if len(waypoint) > 3:
                command_msg.z = waypoint[3]
            else:
                command_msg.z = 0.

            command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            self.waypoint_pub_.publish(command_msg)            

                                     
        # # orientation in quaternion form
        # qw = msg.pose.pose.orientation.w
        # qx = msg.pose.pose.orientation.x
        # qy = msg.pose.pose.orientation.y
        # qz = msg.pose.pose.orientation.z

        # # yaw from quaternion
        # y = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

    def pltOdomCallback(self, msg):
        x = PoseStamped()

        x.header = msg.header
        x.pose = msg.pose.pose

        self.pltPoseCallback(x)
        # self.plt_odom = np.array([msg.pose.pose.position.x,
        #                           -msg.pose.pose.position.y,
        #                           msg.pose.pose.position.z])

    def pltPoseCallback(self, msg):
        current_time = msg.header.stamp.secs+msg.header.stamp.nsecs*1e-9
        dt = current_time - self.plt_prev_time
        self.plt_prev_time = current_time
        self.plt_odom = np.array([msg.pose.position.x,
                                  -msg.pose.position.y,
                                  msg.pose.position.z])

        #numerical differentiation to get velocity
        velocity = (self.plt_odom - self.plt_prev_odom)/dt
        self.plt_prev_odom = self.plt_odom

        x = Odometry()
        x.header.stamp = msg.header.stamp
        x.pose.pose.position.x = self.plt_odom[0]
        x.pose.pose.position.y = self.plt_odom[1]
        x.pose.pose.position.z = self.plt_odom[2]
        x.twist.twist.linear.x = velocity[0]
        x.twist.twist.linear.y = -velocity[1]
        x.twist.twist.linear.z = velocity[2]
        # print('x = ', x)
        self.platform_virtual_odom_pub_.publish(x)

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
