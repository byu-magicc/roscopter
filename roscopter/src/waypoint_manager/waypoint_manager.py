#!/usr/bin/env python3

import numpy as np
import rospy
import csv

# from geometry_msgs import Vector3Stamped
from nav_msgs.msg import Odometry
from roscopter_msgs.msg import Command, PoseEuler
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile, ListWaypoints, \
                               ClearWaypoints, Hold, Release, Land, Fly, ReturnToBase


class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints')
        except KeyError:
            rospy.logfatal('[waypoint_manager] waypoints not set')
            rospy.signal_shutdown('[waypoint_manager] Parameters not set')

        #Initialize variables
        self.n = None
        self.e = None
        self.d = None
        self.psi = None
        self.current_waypoint_index = 0
        self.hold = False
        self.no_command = False
        if len(self.waypoint_list) == 0:
            self.no_command = True
        self.halt_waypoint = [0, 0, 0, 0]
        self.landing = False
        self.landed = False
        self.ready_to_land = False
        self.min_landing_alt = -6 # TODO Positive or Negative?
        self.max_landing_alt = -8 # TODO Positive or Negative?

        # Create the initial poseEuler estimate message
        self.poseEuler_msg = PoseEuler()
        self.poseEuler_msg.n = self.n
        self.poseEuler_msg.e = self.e
        self.poseEuler_msg.d = self.d
        self.poseEuler_msg.psi = self.psi

        # how close does the MAV need to get before going to the next waypoint?
        self.pos_threshold = rospy.get_param('~threshold', 5)
        self.heading_threshold = rospy.get_param('~heading_threshold', 0.035)  # radians
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.print_wp_reached = rospy.get_param('~print_wp_reached', True)

        # Set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.removeWaypointCallback)
        self.set_waypoints_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.setWaypointsFromFileCallback)
        self.list_waypoints_service = rospy.Service('list_waypoints', ListWaypoints, self.listWaypointsCallback)
        self.clear_waypoints_service = rospy.Service('clear_waypoints', ClearWaypoints, self.clearWaypointsCallback)
        self.hold_service = rospy.Service('hold', Hold, self.holdCallback)
        self.release_service = rospy.Service('release', Release, self.releaseCallback)
        self.land_service = rospy.Service('land', Land, self.landCallback)
        self.fly_service = rospy.Service('fly', Fly, self.flyCallback)
        self.rtb_service = rospy.Service('return_to_base', ReturnToBase, self.returnToBaseCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_cmd_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.poseEuler_pub_ = rospy.Publisher('pose_euler', PoseEuler, queue_size=5, latch=True)

        # Wait a second before we publish the first waypoint
        rospy.sleep(2)

        self.poseEuler_pub_.publish(self.poseEuler_msg)

        # Create the initial command message
        self.cmd_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])
        self.publish_command(current_waypoint)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(self, req):
        # This Function adds a waypoint to the waypoint list at the specified index.
        new_waypoint = [req.x, req.y, req.z, req.psi]
        if req.index == -1:
            index = len(self.waypoint_list)
        elif req.index > len(self.waypoint_list) or req.index < -1:
            rospy.logwarn("[waypoint_manager] Waypoint Index Out of Range")
            return False
        # Valid index
        else:
            index = req.index
        self.waypoint_list.insert(index, new_waypoint)
        # Increment the current index if a waypoint is added before the current
        if self.current_waypoint_index > index:
            self.current_waypoint_index += 1
        rospy.loginfo("[waypoint_manager] Added New Waypoint")
        self.no_command = False
        current_waypoint = self.waypoint_list[self.current_waypoint_index]
        self.publish_command(current_waypoint)
        return True

    def removeWaypointCallback(self, req):
        ##### Remove a waypoint at the requested index ####
        #If requested index out of bounds of waypoint list
        if req.index >= len(self.waypoint_list):
            rospy.logwarn("[waypoint_manager] Waypoint Index Out of Range")
            return False
        #If requested index is after the current waypoint index
        elif req.index > self.current_waypoint_index:
            #do nothing
            current_index = self.current_waypoint_index
        #If the current waypoint was removed:
        elif req.index == self.current_waypoint_index:
            #If it's the last waypoint in the list
            last_waypoint_bool = self.current_waypoint_index == (len(self.waypoint_list)-1)
            if last_waypoint_bool:
                #If cyclical, and there remains at least one waypoint
                if self.cyclical_path and len(self.waypoint_list) > 1:
                    current_index = 0
                #If not cyclical, or zero waypoints left
                else:
                    self.no_command = True
                    current_index = 0
                    rospy.sleep(0.1)
                    rospy.loginfo("[waypoint_manager] No remaining commands, pose halted")
            #If not last waypoint in the list
            else:
                current_index = self.current_waypoint_index
        #If the removed waypoint is before the current waypoint index
        else: # req.index < self.current_waypoint_index:
            current_index = self.current_waypoint_index - 1
        #Update private variables and publish command
        self.current_waypoint_index = current_index
        del self.waypoint_list[req.index] # Remove the waypoint
        removed_str = '[waypoint_manager] Waypoint {} Removed'.format(req.index)
        rospy.loginfo(removed_str) # Send loginfo
        if not self.no_command:
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            self.publish_command(current_waypoint)
        return True

    def setWaypointsFromFileCallback(self, req):
        # Sets waypoint list from .csv or .txt file
        if req.Filename.endswith('.csv'):
            with open(req.Filename) as file_wp_list:
                self.waypoint_list = list(csv.reader(file_wp_list, quoting=csv.QUOTE_NONNUMERIC))
        else:
            file = open(req.Filename, 'r')
            file_lines = file.readlines()
            file_wp_list = []
            for waypoint in file_lines:
                file_wp_list.append(map(float, waypoint.strip().split(',')))
            self.waypoint_list = file_wp_list

        # Set index to 0 and publish command to first waypoint
        self.no_command = False
        self.current_waypoint_index = 0
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        self.publish_command(current_waypoint)
        rospy.loginfo("[waypoint_manager] Waypoints Set from File")
        return True

    def listWaypointsCallback(self, req):
        # Returns the waypoint list
        rospy.loginfo('[waypoint_manager] Waypoints:')
        i = 0 # Start index at 0
        for waypoint in self.waypoint_list:
            if i == self.current_waypoint_index:
                waypoint_str = '[waypoint_manager] {}: {} (current_waypoint)'.format(i, waypoint)
            else:
                waypoint_str = '[waypoint_manager] {}: {}'.format(i, waypoint)
            rospy.loginfo(waypoint_str)
            i += 1
        return True

    def clearWaypointsCallback(self, req):
        # Clears all waypoints, except current
        self.no_command = True
        # Wait for Odometry Callback to switch to no_command loop
        rospy.sleep(0.1)
        self.waypoint_list = []
        self.current_waypoint_index = 0
        rospy.loginfo("[waypoint_manager] No remaining commands, pose halted")
        return True

    def holdCallback(self, req):
        # stop and hold current pose
        self.landing = False
        self.landed = False
        self.holdPose()
        return True

    def holdPose(self):
        self.hold = True
        rospy.loginfo("[waypoint_manager] Multirotor's pose is held - release to continue")

    def releaseCallback(self, req):
        if self.hold == False:
            rospy.logwarn("[waypoint_manager] Cannot release - Not in hold")
            return False
        elif len(self.waypoint_list) == 0:
            rospy.logwarn("[waypoint_manager] Cannot release - Zero Waypoints")
            return False
        else:
            self.hold = False
            current_waypoint = self.waypoint_list[self.current_waypoint_index]
            self.publish_command(current_waypoint)
            rospy.loginfo("[waypoint_manager] Released hold - Multirotor path in progress")
            return True

    def landCallback(self, req):
        if self.hold == True:
            rospy.loginfo("[waypoint_manager] Releasing hold to land")
            self.hold == False
        rospy.loginfo("[waypoint_manager] Preparing to land at coordinates [{} {}]".format(self.n,self.e))
        landing_alt = min(max(self.d, self.max_landing_alt),self.min_landing_alt)
        self.landing_pose = [self.n, self.e, landing_alt, self.psi]
        self.landing = True
        return True

    def returnToBaseCallback(self, req):
        if self.hold == True:
            rospy.loginfo("[waypoint_manager] Releasing hold to return to base")
            self.hold == False
        rospy.loginfo("[waypoint_manager] Preparing to land at base coordinates [0 0]")
        landing_alt = min(max(self.d, self.max_landing_alt),self.min_landing_alt)
        self.landing_pose = [0, 0, landing_alt, 0]
        self.landing = True
        return True

    def prepare_to_land(self):
        current_position = np.array([self.n, self.e, self.d])
        position_error = np.linalg.norm(self.landing_pose[0:3] - current_position)
        heading_error = np.abs(self.wrap(self.landing_pose[3] - self.psi))
        if position_error < self.pos_threshold and heading_error < self.heading_threshold:
            self.ready_to_land = True
            rospy.loginfo("[waypoint_manager] Landing at coordinates [{} {}]".format(self.landing_pose[0], self.landing_pose[1]))
        else:
            self.publish_command(self.landing_pose)
        return

    def land(self):
        #TODO: Make these into params
        approach_vel = 0.8 # m/s, velocity before reaching the slow_d
        landing_vel = 0.4 # m/s, velocity when hitting the ground
        slow_d = -1.2 # The altitude to start slowing the quad
        stop_d = -0.05 # The altitude to stop the quad at

        self.cmd_msg.stamp = rospy.Time.now()
        self.cmd_msg.cmd1 = self.landing_pose[0]
        self.cmd_msg.cmd2 = self.landing_pose[1]
        self.cmd_msg.cmd4 = self.landing_pose[3]
        if self.d < stop_d:
            if self.d < slow_d:
                self.cmd_msg.cmd3 = approach_vel
            else:
                self.cmd_msg.cmd3 = -approach_vel/(1 + landing_vel/approach_vel/(-landing_vel/approach_vel + 1) \
                                    + np.exp(10/(slow_d-stop_d)*(self.d - (slow_d+stop_d)/2))) + approach_vel
            self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DVEL_YAW
            self.waypoint_cmd_pub_.publish(self.cmd_msg)
        elif self.landed == False:
            self.cmd_msg.mode = Command.MODE_NACC_EACC_DACC_YAWRATE
            self.cmd_msg.cmd1 = 0.0
            self.cmd_msg.cmd2 = 0.0
            self.cmd_msg.cmd3 = 10
            self.cmd_msg.cmd4 = 0.0
            self.waypoint_cmd_pub_.publish(self.cmd_msg)
            self.landed = True
            rospy.loginfo("[waypoint_manager] Landed")
        return

    def flyCallback(self,req):
        if self.landing == False:
            rospy.loginfo("[waypoint_manager] Already Flying")
            return False
        self.ready_to_land = False
        self.landing = False
        self.landed = False
        rospy.loginfo("[waypoint_manager] Resuming Flight")
        current_waypoint = self.waypoint_list[self.current_waypoint_index]
        self.publish_command(current_waypoint)
        return True

    def odometryCallback(self, msg):
        ###### Retrieve and Publish the Current Pose ######
        # get current position
        self.n = msg.pose.pose.position.x
        self.e = msg.pose.pose.position.y
        self.d = msg.pose.pose.position.z
        current_position = np.array([self.n, self.e, self.d])

        # orientation in quaternion form
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        # yaw from quaternion
        self.psi = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

        # publish pose Euler estimate
        self.poseEuler_msg.n = self.n
        self.poseEuler_msg.e = self.e
        self.poseEuler_msg.d = self.d
        self.poseEuler_msg.psi = self.psi
        self.poseEuler_pub_.publish(self.poseEuler_msg)

        # Don't command/track waypoints if landing
        if self.landing == True:
            if self.landed == False:
                self.halt_waypoint = [self.n , self.e , self.d , self.psi ]
                if self.ready_to_land == False:
                    self.prepare_to_land()
                else:
                    self.land()
            return

        ###### Halt Pose - If Hold or No Command ######
        elif self.hold or self.no_command:
            self.publish_command(self.halt_waypoint)
            return

        ###### Check Waypoint Arrival Status & Update to Next Waypoint #######
        else:
            # Calculate error between current pose and commanded waypoint
            self.halt_waypoint = [self.n , self.e , self.d , self.psi ]
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

            position_error = np.linalg.norm(current_waypoint[0:3] - current_position)
            heading_error = np.abs(self.wrap(current_waypoint[3] - self.psi))
            #if error is within the threshold
            if position_error < self.pos_threshold and heading_error < self.heading_threshold:
                #Print if we did not already print
                if self.print_wp_reached:
                    idx = self.current_waypoint_index
                    rospy.loginfo('[waypoint_manager] Reached waypoint {}'.format(idx))
                # stop iterating over waypoints if cycle==false and last waypoint reached
                if not self.cyclical_path and self.current_waypoint_index == len(self.waypoint_list)-1:
                    self.no_command = True
                    rospy.loginfo("[waypoint_manager] No remaining commands, pose halted")
                    return
                # Get new waypoint index #TODO: If cyclical == True and only one waypoint, don't keep printing
                else:
                    self.current_waypoint_index += 1
                    self.current_waypoint_index %= len(self.waypoint_list)
                    current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                    self.publish_command(current_waypoint)
                    return

    def publish_command(self, current_waypoint):
        self.cmd_msg.stamp = rospy.Time.now()
        self.cmd_msg.cmd1 = current_waypoint[0]
        self.cmd_msg.cmd2 = current_waypoint[1]
        self.cmd_msg.cmd3 = current_waypoint[2]

        if len(current_waypoint) > 3:
            self.cmd_msg.cmd4 = current_waypoint[3]
        else:
            next_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoint_list)
            next_waypoint = self.waypoint_list[next_waypoint_index]
            delta = next_waypoint - current_waypoint
            self.cmd_msg.cmd4 = np.arctan2(delta[1], delta[0])

        self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
        self.waypoint_cmd_pub_.publish(self.cmd_msg)
        return

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
