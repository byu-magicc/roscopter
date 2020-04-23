#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



class PlatformManager():

    def __init__(self):

        # get parameters
        try:
            self.velocity_list = rospy.get_param('~linear_velocity') #params are loaded in launch file
            self.angular_velocity_list = rospy.get_param('~angular_velocity') #params are loaded in launch file
            self.transition_time = rospy.get_param('~transition_time')
        except KeyError:
            rospy.logfatal('velocities not set')
            rospy.signal_shutdown('Parameters not set')

        # Wait a second before we publish the first waypoint
        # while (rospy.Time.now() < rospy.Time(15.)):
        while (rospy.Time.now() < rospy.Time(3.)):
            pass

        # Set Up Publishers and Subscribers
        self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odometryCallback, queue_size=5)
        self.twist_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5, latch=True)

        self.twist_msg = Twist()

        velocity = self.velocity_list[0]
        angular_velocity = self.angular_velocity_list[0]

        self.twist_msg.linear.x = velocity[0]
        self.twist_msg.linear.y = velocity[1]
        self.twist_msg.linear.z = velocity[2]
        self.twist_msg.angular.x = angular_velocity[0]
        self.twist_msg.angular.y = angular_velocity[1]
        self.twist_msg.angular.z = angular_velocity[2]

        self.twist_pub_.publish(self.twist_msg)

        #just make sure the node doesn't shut down
        while not rospy.is_shutdown():
            rospy.spin()

    def odometryCallback(self, msg):
        time = msg.header.stamp.secs+msg.header.stamp.nsecs*1E-9
        intervals = np.floor(time/self.transition_time)
        velocity_index = int(intervals%len(self.velocity_list))
        velocity = self.velocity_list[velocity_index]
        angular_velocity = self.angular_velocity_list[velocity_index]

        self.twist_msg.linear.x = velocity[0]
        self.twist_msg.linear.y = velocity[1]
        self.twist_msg.linear.z = velocity[2]
        self.twist_msg.angular.x = angular_velocity[0]
        self.twist_msg.angular.y = angular_velocity[1]
        self.twist_msg.angular.z = angular_velocity[2]

        self.twist_pub_.publish(self.twist_msg)


if __name__ == '__main__':
    rospy.init_node('platform_manager', anonymous=True)
    try:
        p_manager = PlatformManager()
    except:
        rospy.ROSInterruptException
    pass
