#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from rosflight_msgs.msg import GNSS
from roscopter_msgs.msg import RelativePose
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from ublox.msg import RelPos
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


class SimManager():

    def __init__(self):

        self.plt_pos = np.zeros(3)
        self.drone_pos = np.zeros(3)
        self.relPos = RelPos()

        # Set Up Publishers and Subscribers
        # self.platform_virtual_odom_pub_ = rospy.Publisher('platform_virtual_odometry', Odometry, queue_size=5, latch=True)
        self.relpos_pub_ = rospy.Publisher('relpos', RelPos, queue_size=5, latch=True)        
        self.plt_odom_sub_ = rospy.Subscriber('platform_odom', Odometry, self.pltOdomCallback, queue_size=5)
        self.drone_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.droneOdomCallback, queue_size=5)
        self.gnss_sub_ = rospy.Subscriber("gnss", GNSS, self.gnssCallback, queue_size=5)

        
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def pltOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.plt_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])


    def droneOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.drone_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])

    def gnssCallback(self, msg):
        self.publish_virtual_relPos()

    def publish_virtual_relPos(self):
        relPos_array = self.drone_pos - self.plt_pos
        self.relPos.relPosNED[0] = relPos_array[0]
        self.relPos.relPosNED[1] = relPos_array[1]
        self.relPos.relPosNED[2] = relPos_array[2]

        self.relpos_pub_.publish(self.relPos)     
        


if __name__ == '__main__':
    rospy.init_node('sim_manager', anonymous=True)
    try:
        sim_manager = SimManager()
    except:
        rospy.ROSInterruptException
    pass
