#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped


class WaypointManager():

    def __init__(self):

        #load parameters
        self.load_set_parameters()

        # Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Publishers
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)

        #Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        
        # Wait a second before we publish the first waypoint
        while (rospy.Time.now() < rospy.Time(2.)):
            pass

        # publish first waypoint
        current_waypoint = np.array(self.waypoint_list[0])
        self.new_waypoint(current_waypoint)

        while not rospy.is_shutdown():
            rospy.spin()


    #TODO: Need to set up these services
    def addWaypointCallback(req):
        print("addwaypoints")
        

    def removeWaypointCallback(self, req):
        #TODO
        print("[waypoint_manager] remove Waypoints (NOT IMPLEMENTED)")


    def setWaypointsFromFile(self, req):
        #TODO
        print("[waypoint_manager] set Waypoints from File (NOT IMPLEMENTED)")


    def odometryCallback(self, msg):
        
        #waypoints are in neu
        current_position_neu = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
        
        #not using orientation right now
        # current_orient = [msg.pose.pose.orientation.x,
        #                     msg.pose.pose.orientation.y,
        #                     msg.pose.pose.orientation.z,
        #                     msg.pose.pose.orientation.w]
        # # yaw from quaternion
        # qx = current_orient[0]
        # qy = current_orient[1]
        # qz = current_orient[2]
        # qw = current_orient[3]
        # y = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        # #publish the relative pose estimate
        # relativePose_msg = RelativePose()
        # relativePose_msg.x = current_position[0]
        # relativePose_msg.y = current_position[1]
        # relativePose_msg.z = y
        # relativePose_msg.F = current_position[2]
        # self.relPose_pub_.publish(relativePose_msg)

        self.update(current_position_neu)           

    
    def update(self, current_position):

        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

        error = np.linalg.norm(current_position - current_waypoint[0:3])
        
        #may want to implement heading error at some point
        # heading_error = np.abs(self.wrap(current_waypoint[3] - y))

        if error < self.threshold:
            print'reached waypoint ', self.current_waypoint_index + 1
            # Get new waypoint index
            self.current_waypoint_index += 1

            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)            
            elif self.current_waypoint_index == len(self.waypoint_list):
                self.current_waypoint_index -=1

            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            self.new_waypoint(next_waypoint)

    def new_waypoint(self, waypoint):

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.x = waypoint[0]
        self.cmd_msg.y = waypoint[1]
        self.cmd_msg.F = waypoint[2]

        if len(waypoint) > 3:
            self.cmd_msg.z = waypoint[3]
        else:
            self.cmd_msg.z = 0.

        self.cmd_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(self.cmd_msg)

    
    def load_set_parameters(self):
        
        try:
            self.waypoint_list = rospy.get_param('~waypoints') #params are loaded in launch file
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')
        self.threshold = rospy.get_param('~threshold', 5)
        self.cyclical_path = rospy.get_param('~cycle', False)
        self.print_wp_reached = rospy.get_param('~print_wp_reached', True)

        #calculate parameters
        self.len_wps = len(self.waypoint_list)
        self.prev_time = rospy.Time.now()

        #other variables and arrays
        self.current_waypoint_index = 0
        self.current_waypoint_index = 0

        #message types
        self.cmd_msg = Command()

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
