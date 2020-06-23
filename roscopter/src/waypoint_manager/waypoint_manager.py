#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.msg import RelativePose
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
# from ublox.msg import RelPos


class WaypointManager():

    def __init__(self):
        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints') #params are loaded in launch file
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')

        #initialize platform location
        # self.plt_odom = np.array([0.0,
        #                           0.0,
        #                           0.0])
        # self.plt_prev_odom = np.array([0.0,
        #                                0.0,
        #                                0.0])

        self.drone_odom = np.zeros(3)
        self.plt_pos = np.zeros(3)
                                       
        self.len_wps = len(self.waypoint_list)
        self.current_waypoint_index = 0

        # how close does the MAV need to get before going to the next waypoint?

        self.threshold = rospy.get_param('~threshold', 5)
        self.landing_threshold = rospy.get_param('~landing_threshold', 1)
        self.begin_descent_height = rospy.get_param('~begin_descent_height', 2)
        self.begin_landing_height = rospy.get_param('~begin_landing_height', 0.2)
        self.cyclical_path = rospy.get_param('~cycle', False)
        self.auto_land = rospy.get_param('~auto_land', False)
        self.print_wp_reached = rospy.get_param('~print_wp_reached', True)

        self.mission_state = 0 #0: mission
                               #1: rendevous
                               #2: descend
                               #3: land

        self.prev_time = rospy.Time.now()
        self.plt_prev_time = 0.0
        self.is_landing = 0
        self.current_waypoint_index = 0

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Wait a second before we publish the first waypoint
        while (rospy.Time.now() < rospy.Time(2.)):
            pass

        # Create the initial command message
        self.cmd_msg = Command()
        
        current_waypoint = np.array(self.waypoint_list[0])

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.x = current_waypoint[0]
        self.cmd_msg.y = current_waypoint[1]
        self.cmd_msg.F = current_waypoint[2]

        if len(current_waypoint) > 3:
            self.cmd_msg.z = current_waypoint[3]
        else:
            self.cmd_msg.z = 0.
        self.cmd_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE

        # Set Up Publishers and Subscribers
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.auto_land_pub_ = rospy.Publisher('auto_land', Bool, queue_size=5, latch=True)
        self.is_landing_pub_ = rospy.Publisher('is_landing', Bool, queue_size=5, latch=True)
        self.landed_pub_ = rospy.Publisher('landed', Bool, queue_size=5, latch=True)
        # self.platform_virtual_odom_pub_ = rospy.Publisher('platform_virtual_odometry', Odometry, queue_size=5, latch=True)
        self.error_pub_ = rospy.Publisher('error', Pose, queue_size=5, latch=True)
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.plt_relPos_sub_ = rospy.Subscriber('plt_relPos', PointStamped, self.pltRelPosCallback, queue_size=5)
        # self.drone_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.droneOdomCallback, queue_size=5)
        # Wait a second before we publish the first waypoint
        # rospy.sleep(2)

        # Create the initial relPose estimate message
        # relativePose_msg = RelativePose()
        # relativePose_msg.x = 0
        # relativePose_msg.y = 0
        # relativePose_msg.z = 0
        # relativePose_msg.F = 0
        # self.relPose_pub_.publish(relativePose_msg)

        self.waypoint_pub_.publish(self.cmd_msg)
        
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
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
        # Get error between waypoint and current state
        self.drone_odom = np.array([msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z])
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
        
        # current_orient = [msg.pose.pose.orientation.x,
        #                     msg.pose.pose.orientation.y,
        #                     msg.pose.pose.orientation.z,
        #                     msg.pose.pose.orientation.w]

        if self.mission_state == 1:
            self.rendevous(current_position)
        elif self.mission_state == 2:
            self.descend(current_position)
        elif self.mission_state == 3:
            self.land(current_position)
        else:
            self.mission(current_position)           

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
    
    def mission(self, current_position):
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

        self.publish_error(current_position, current_waypoint)
        error = np.linalg.norm(current_position - current_waypoint[0:3])
        
        #may want to implement heading error at some point
        # heading_error = np.abs(self.wrap(current_waypoint[3] - y))

        if error < self.threshold:
            print'reached waypoint ', self.current_waypoint_index + 1
            # Get new waypoint index
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.waypoint_list) and self.auto_land == True:
                self.mission_state = 1 #switch to rendevous state
                print('rendevous state')
                return

            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)            
            elif self.current_waypoint_index == len(self.waypoint_list):
                self.current_waypoint_index -=1

            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            self.new_waypoint(next_waypoint)

    def rendevous(self, current_position):

        self.auto_land_pub_.publish(True) #this will signal the controller to include the velocity feed forward term from the barge

        waypoint = self.plt_pos + np.array([0.0, 0.0, self.begin_descent_height])
        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)

        self.new_waypoint(waypoint)

        if error < self.threshold:
            self.mission_state = 2 #switch to descent state
            print('descend state')

    def descend(self, current_position):

        waypoint = self.plt_pos + np.array([0.0, 0.0, self.begin_landing_height])
        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)

        self.new_waypoint(waypoint)

        if error < self.landing_threshold:
            self.mission_state = 3 #switch to land state
            print('land state')

    def land(self, current_position):

        waypoint =self.plt_pos
        if self.is_landing == 0:
            self.new_waypoint(waypoint)
            self.is_landing = 1
            self.is_landing_pub_.publish(True) #this will signal the controller to include the velocity feed forward term from the barge

        error = np.linalg.norm(current_position - waypoint)
        self.publish_error(current_position, waypoint)
        if error < self.threshold:
            self.landed_pub_.publish(True)
            #TODO find a way to disarm after reaching the waypoint

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


    def pltRelPosCallback(self, msg):
        #TODO: implement time for the plt_relPos message?

        #flip to NEU
        self.plt_pos[0] = msg.point.x + self.drone_odom[0]
        self.plt_pos[1] = msg.point.y + self.drone_odom[1]
        self.plt_pos[2] = -msg.point.z - self.drone_odom[2]

    # def droneOdomCallback(self, msg):
    #     self.drone_odom = np.array([msg.pose.pose.position.x,
    #                                 msg.pose.pose.position.y,
    #                                 msg.pose.pose.position.z])        

    def publish_error(self, current_position, current_waypoint):
        error_msg = Pose()
        error_msg.position.x = current_position[0] - current_waypoint[0]
        error_msg.position.y = current_position[1] - current_waypoint[1]
        error_msg.position.z = current_position[2] - current_waypoint[2]
        self.error_pub_.publish(error_msg)

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
