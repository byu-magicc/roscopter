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
        self.rover_virtual_mocap_ned = PoseStamped()

        self.origin_set = False
        self.origin = np.zeros(3)

        mocap_rate = 100 #hz        

        # # Set Up Publishers and Subscribers
        self.rover_virtual_mocap_ned_pub_ = rospy.Publisher('rover_mocap', PoseStamped, queue_size=5, latch=True)
        self.rover_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.roverOdomCallback, queue_size=5)
    
        rate = rospy.Rate(mocap_rate)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            # rospy.spin()
            rate.sleep()


    def roverOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.rover_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])
        if self.origin_set == False:
            self.origin = self.rover_pos
            self.origin_set = True

        self.publish_rover_virtual_mocap_ned()


    def publish_rover_virtual_mocap_ned(self):
        rover_virtual_mocap_ned_array = self.rover_pos - self.origin

        self.rover_virtual_mocap_ned.header.stamp = rospy.Time.now()
        self.rover_virtual_mocap_ned.pose.position.x = rover_virtual_mocap_ned_array[0]
        self.rover_virtual_mocap_ned.pose.position.y = rover_virtual_mocap_ned_array[1]
        self.rover_virtual_mocap_ned.pose.position.z = rover_virtual_mocap_ned_array[2]
                
        self.rover_virtual_mocap_ned_pub_.publish(self.rover_virtual_mocap_ned)


if __name__ == '__main__':
    rospy.init_node('mocap_sim_manager', anonymous=True)
    try:
        mocap_sim_manager = MocapSimManager()
    except:
        rospy.ROSInterruptException
    pass
