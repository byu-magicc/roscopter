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

        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 5)
        self.begin_descent_height = rospy.get_param('~begin_descent_height', 2)
        self.begin_landing_height = rospy.get_param('~begin_landing_height', 0.2)
        self.cyclical_path = rospy.get_param('~cycle', False)

        self.mission_state = 0 #0: mission
                               #1: rendevous
                               #2: descend
                               #3: land

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
        self.auto_land_pub_ = rospy.Publisher('auto_land', Bool, queue_size=5, latch=True)
        self.platform_virtual_odom_pub_ = rospy.Publisher('platform_virtual_odometry', Odometry, queue_size=5, latch=True)
        
        self.auto_land = rospy.get_param('~auto_land', False)
        self.current_waypoint_index = 0

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

        if self.mission_state == 1:
            self.rendevous(current_position)
        elif self.mission_state == 2:
            self.descend(current_position)
        elif self.mission_state == 3:
            self.land(current_position)
        else:
            self.mission(current_position)           

        # # orientation in quaternion form
        # qw = msg.pose.pose.orientation.w
        # qx = msg.pose.pose.orientation.x
        # qy = msg.pose.pose.orientation.y
        # qz = msg.pose.pose.orientation.z

        # # yaw from quaternion
        # y = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    
    def mission(self, current_position):

        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        error = np.linalg.norm(current_position - current_waypoint[0:3])
       
        if error < self.threshold:
            # Get new waypoint index
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.waypoint_list) and self.auto_land == True:
                self.mission_state = 1 #switch to rendevous state
                return

            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)                
            elif self.current_waypoint_index == len(self.waypoint_list):
                self.current_waypoint_index -=1

            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            self.new_waypoint(next_waypoint)

    def rendevous(self, current_position):

        self.auto_land_pub_.publish(True) #this will signal the controller to include the velocity feed forward term from the barge

        waypoint = self.plt_odom + np.array([0.0, 0.0, self.begin_descent_height])
        error = np.linalg.norm(current_position - waypoint)

        self.new_waypoint(waypoint)

        if error < self.threshold:
            self.mission_state = 2 #switch to descent state

    def descend(self, current_position):

        waypoint = self.plt_odom + np.array([0.0, 0.0, self.begin_landing_height])
        error = np.linalg.norm(current_position - waypoint)

        self.new_waypoint(waypoint)

        if error < self.threshold:
            self.mission_state = 3 #switch to land state

    def land(self, current_position):
        a = 1

    def new_waypoint(self, waypoint):
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
