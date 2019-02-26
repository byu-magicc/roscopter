#!/usr/bin/env python

import numpy as np

import rospy, tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from std_srvs.srv import Trigger


class CarFollower():

    def __init__(self):

        self._state = 'None'
        self._landing_now = False
        self._landing_now_thresh = 0.1

        # Set Up Publishers and Subscribers
        self._take_off_srv = rospy.Service('take_off', Trigger,
                self.takeOffCallback)
        self._land_srv = rospy.Service('land', Trigger,
                self.landCallback)
        self._car_srv = rospy.Service('car', Trigger,
                self.carSrvCallback)
        self._car_mocap_sub = rospy.Subscriber('car_mocap', PoseStamped,
                self.carMocapCallback, queue_size=5)
        self._uav_odom_sub = rospy.Subscriber('odom', Odometry,
                self.uavOdomCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def carMocapCallback(self, msg):
        if self._state == 'car':
            commanded_alt = -1.5
            commanded_heading = 0.0

            commanded_north = msg.pose.position.x
            commanded_east = msg.pose.position.z
        elif self._state == 'take_off':
            commanded_alt = -1.5
            commanded_heading = 0.0
            commanded_north = 0.0
            commanded_east = 0.0
        elif self._state == 'land':
            if self._landing_now:
                commanded_alt = 0.0
                commanded_heading = 0.0
                commanded_north = 0.0
                commanded_east = 0.0
            else:
                commanded_alt = -1.5
                commanded_heading = 0.0
                commanded_north = 0.0
                commanded_east = 0.0

        if self._state != 'None':
            waypoint_cmd = Command()
            waypoint_cmd.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            waypoint_cmd.x = commanded_north
            waypoint_cmd.y = commanded_east
            waypoint_cmd.z = commanded_heading
            waypoint_cmd.F = commanded_alt

            self.waypoint_pub_.publish(waypoint_cmd)

    def uavOdomCallback(self, msg):
        if self._state == 'land':
            uav_north = msg.pose.pose.position.x
            uav_east = msg.pose.pose.position.y
            uav_err = np.sqrt(uav_north**2. + uav_east**2.)

            if uav_err < self._landing_now_thresh:
                self._landing_now = True

    def takeOffCallback(self, req):
        if self._state == 'None':
            self._state = 'take_off'
            return True, "success"
        else:
            return False, "Fail"

    def landCallback(self, req):
        if self._state == 'car':
            self._state = 'land'
            return True, "success"
        else:
            return False, "Fail"

    def carSrvCallback(self, req):
        if self._state == 'take_off':
            self._state = 'car'
            return True, "success"
        else:
            return False, "Fail"

if __name__ == '__main__':
    rospy.init_node('car_follower', anonymous=True)
    try:
        wp_manager = CarFollower()
    except:
        rospy.ROSInterruptException
    pass
