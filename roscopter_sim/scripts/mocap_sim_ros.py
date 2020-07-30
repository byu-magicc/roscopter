#!/usr/bin/env python

import numpy as np
import rospy
import std_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import GNSS
from ublox.msg import PosVelEcef


class MocapSimManager():


    def __init__(self):

        self.rover_pos = np.zeros(3)
        self.rover_orient = np.zeros(4)
        self.rover_virtual_mocap_ned = PoseStamped()

        self.origin_set = False
        self.origin = np.zeros(3)

        #TODO implement this
        mocap_rate = 100 #hz 
        mocap_period = 1.0/mocap_rate       

        # # Set Up Publishers and Subscribers
        self.rover_virtual_mocap_ned_pub_ = rospy.Publisher('rover_mocap', PoseStamped, queue_size=5, latch=True)
        self.rover_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.roverOdomCallback, queue_size=5)

        # Timer
        self.mocap_rate_timer_ = rospy.Timer(rospy.Duration(mocap_period), self.mocapRateCallback)
    
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def roverOdomCallback(self, msg):


        self.rover_virtual_mocap_ned.pose.position.x = msg.pose.pose.position.x
        self.rover_virtual_mocap_ned.pose.position.y = msg.pose.pose.position.y
        self.rover_virtual_mocap_ned.pose.position.z = msg.pose.pose.position.z
        self.rover_virtual_mocap_ned.pose.orientation.x = msg.pose.pose.orientation.x
        self.rover_virtual_mocap_ned.pose.orientation.y = msg.pose.pose.orientation.y
        self.rover_virtual_mocap_ned.pose.orientation.z = msg.pose.pose.orientation.z
        self.rover_virtual_mocap_ned.pose.orientation.w = msg.pose.pose.orientation.w

    
    def mocapRateCallback(self, event):
        
        self.rover_virtual_mocap_ned.header.stamp = rospy.Time.now()                
        self.rover_virtual_mocap_ned_pub_.publish(self.rover_virtual_mocap_ned)


if __name__ == '__main__':
    rospy.init_node('mocap_sim_manager', anonymous=True)
    try:
        mocap_sim_manager = MocapSimManager()
    except:
        rospy.ROSInterruptException
    pass
