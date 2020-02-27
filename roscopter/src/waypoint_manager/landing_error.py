#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry



class LandingError():

    def __init__(self):
        self.platform_odom = Odometry

        # Set Up Publishers and Subscribers
        self.platform_odom_sub_ = rospy.Subscriber('odom', Odometry, self.platformOdometryCallback, queue_size=5)
        self.drone_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.droneOdometryCallback, queue_size=5)
        self.error_pub_ = rospy.Publisher('landing_error', Odometry, queue_size=5, latch=True)

        self.error_msg = Odometry()

        #just make sure the node doesn't shut down
        while not rospy.is_shutdown():
            rospy.spin()

    def platformOdometryCallback(self, msg):
        self.platform_odom = msg

    def droneOdometryCallback(self, msg):
        self.error_msg.pose.pose.position.x = self.platform_odom.pose.pose.position.x - msg.pose.pose.position.x
        self.error_msg.pose.pose.position.y = self.platform_odom.pose.pose.position.y + msg.pose.pose.position.y
        self.error_msg.pose.pose.position.z = self.platform_odom.pose.pose.position.z - msg.pose.pose.position.z
        self.error_pub_.publish(self.error_msg)


if __name__ == '__main__':
    rospy.init_node('landing_error', anonymous=True)
    try:
        l_error = LandingError()
    except:
        rospy.ROSInterruptException
    pass
