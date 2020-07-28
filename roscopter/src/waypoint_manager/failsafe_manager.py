#!/usr/bin/env python3

import numpy as np
import rospy

from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Status
from roscopter_msgs.msg import Command, Failsafe

class FailsafeManager():

    def __init__(self):
        # ROSflight Failsafe Variables
        self.rf_failsafe = False
        self.rf_failsafe_mode = 'RETURN_TO_BASE' # This will be a fixed parameter
        self.rf_waypoint_set = False

        # External Failsafe Variables
        self.ext_failsafe = False
        self.ext_failsafe_mode = 'RETURN_TO_BASE' # This will come from the failsafe topic
        self.ext_waypoint_set = False

        # Waypoint Variables
        self.pos_threshold = rospy.get_param('~threshold', 1)
        self.heading_threshold = rospy.get_param('~heading_threshold', 0.35)  # radians

        self.ready_to_land = False
        self.landed = False
        self.cmd_msg = Command()

        # Landing Parameters
        self.min_landing_alt = rospy.get_param('~min_landing_alt', -1)
        self.max_landing_alt = rospy.get_param('~max_landing_alt', -6)
        self.approach_vel = rospy.get_param('~approach_vel', 0.8)
        self.landing_vel = rospy.get_param('~landing_vel', 0.4)
        self.slow_alt = rospy.get_param('~slow_alt', -1.2)
        self.stop_alt = rospy.get_param('~stop_alt', -0.05)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.status_sub_ = rospy.Subscriber('status', Status, self.statusCallback, queue_size=10)
        self.failsafe_sub_ = rospy.Subscriber('failsafe', Failsafe, self.failsafeCallback, queue_size=10)

        self.command_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.failsafe_command_pub_ = rospy.Publisher('fs_high_level_command', Command, queue_size=5, latch=True)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def statusCallback(self, msg):
        if msg.failsafe == False:
            self.rf_failsafe = False
            self.rf_waypoint_set = False
            return
        else:
            self.rf_failsafe = True
            return

    def failsafeCallback(self, msg):
        if msg.failsafe == False: # or self.failsafe == True ??
            # self.ext_failsafe = False # Commented out to ensure that the copter will return to base and land.
            # self.ext_waypoint_set = False
            return
        else:
            self.ext_failsafe = True
            if msg.mode == Failsafe.RETURN_TO_BASE:
                self.ext_failsafe_mode = 'RETURN_TO_BASE'
                return
            else:
                self.ext_failsafe_mode = 'LAND'
                return

    def set_waypoint(self, mode):
        if mode == 'rf':
            if self.rf_failsafe_mode == 'RETURN_TO_BASE':
                landing_alt = min(max(self.d, self.max_landing_alt),self.min_landing_alt)
                self.rf_landing_pose = [0, 0, landing_alt, 0]
            else:
                landing_alt = min(max(self.d, self.max_landing_alt),self.min_landing_alt)
                self.rf_landing_pose = [self.n, self.e, landing_alt, self.psi]
            self.rf_waypoint_set = True
            rospy.loginfo("[failsafe_manager] Following ROSflight Failsafe Mode: {}".format(self.rf_failsafe_mode))
        elif mode == 'ext':
            if self.ext_failsafe_mode == 'RETURN_TO_BASE':
                landing_alt = min(max(self.d, self.max_landing_alt),self.min_landing_alt)
                self.ext_landing_pose = [0, 0, landing_alt, 0]
            else:
                landing_alt = min(max(self.d, self.max_landing_alt),self.min_landing_alt)
                self.ext_landing_pose = [self.n, self.e, landing_alt, self.psi]
            self.ext_waypoint_set = True
            rospy.loginfo("[failsafe_manager] Following External Failsafe Mode: {}".format(self.ext_failsafe_mode))
        return

    def prepare_to_land(self):
        if self.rf_failsafe == True:
            landing_pose = self.rf_landing_pose
        else:
            landing_pose = self.ext_landing_pose
        current_position = np.array([self.n, self.e, self.d])
        position_error = np.linalg.norm(landing_pose[0:3] - current_position)
        heading_error = np.abs(self.wrap(landing_pose[3] - self.psi))
        if position_error < self.pos_threshold and heading_error < self.heading_threshold:
            self.ready_to_land = True
            rospy.loginfo("[failsafe_manager] Landing at coordinates [{} {}]".format(landing_pose[0], landing_pose[1]))
        else:
            self.publish_wp_command(landing_pose)
        return

    def land(self):
        if self.rf_failsafe == True:
            landing_pose = self.rf_landing_pose
        else:
            landing_pose = self.ext_landing_pose
        self.cmd_msg.stamp = rospy.Time.now()
        self.cmd_msg.cmd1 = landing_pose[0]
        self.cmd_msg.cmd2 = landing_pose[1]
        self.cmd_msg.cmd4 = landing_pose[3]
        if self.d < self.stop_alt:
            if self.d < self.slow_alt:
                self.cmd_msg.cmd3 = self.approach_vel
            else:
                self.cmd_msg.cmd3 = -self.approach_vel/(1 + self.landing_vel/self.approach_vel/(-self.landing_vel/self.approach_vel + 1) \
                                    + np.exp(10/(self.slow_alt-self.stop_alt)*(self.d - (self.slow_alt+self.stop_alt)/2))) + self.approach_vel
            self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DVEL_YAW
            self.publish_command_msg()
        elif self.landed == False:
            self.cmd_msg.mode = Command.MODE_NACC_EACC_DACC_YAWRATE
            self.cmd_msg.cmd1 = 0.0
            self.cmd_msg.cmd2 = 0.0
            self.cmd_msg.cmd3 = 1.0
            self.cmd_msg.cmd4 = 0.0
            self.publish_command_msg()
            self.landed = True
            rospy.loginfo("[failsafe_manager] Landed, disarm now")
        return

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

        if self.landed == False:
        # ROSflight failsafe takes priority
            if self.rf_failsafe == True:
                if self.rf_waypoint_set == False:
                    self.set_waypoint('rf')
                else:
                    if self.ready_to_land == False:
                        self.prepare_to_land()
                    else:
                        self.land()
            elif self.ext_failsafe == True:
                if self.ext_waypoint_set == False:
                    self.set_waypoint('ext')
                else:
                    if self.ready_to_land == False:
                        self.prepare_to_land()
                    else:
                        self.land()
        return

    def publish_wp_command(self, current_waypoint):
        self.cmd_msg.stamp = rospy.Time.now()
        self.cmd_msg.cmd1 = current_waypoint[0]
        self.cmd_msg.cmd2 = current_waypoint[1]
        self.cmd_msg.cmd3 = current_waypoint[2]
        self.cmd_msg.cmd4 = current_waypoint[3]

        self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
        self.publish_command_msg()
        return

    def publish_command_msg(self):
        if self.rf_failsafe == True:
            self.failsafe_command_pub_.publish(self.cmd_msg)
        if self.ext_failsafe == True:
            self.command_pub_.publish(self.cmd_msg)
        return

    def wrap(self, angle):
        angle -= 2*np.pi * np.floor((angle + np.pi) / (2*np.pi))
        return angle

if __name__ == '__main__':
    rospy.init_node('failsafe_manager', anonymous=True)
    try:
        fs_manager = FailsafeManager()
    except:
        rospy.ROSInterruptException
    pass
