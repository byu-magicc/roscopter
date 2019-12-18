#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist


class PlatformManager():

    def __init__(self):

        # get parameters
        try:
            velocity_list = rospy.get_param('~linear_velocity') #params are loaded in launch file
            angular_velocity_list = rospy.get_param('~angular_velocity') #params are loaded in launch file
        except KeyError:
            rospy.logfatal('velocities not set')
            rospy.signal_shutdown('Parameters not set')

        # Wait a second before we publish the first waypoint
        while (rospy.Time.now() < rospy.Time(15.)):
            pass

        # Set Up Publishers
        twist_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5, latch=True)

        twist_msg = Twist()
        
        twist_msg.linear.x = velocity_list[0]
        twist_msg.linear.y = velocity_list[1]
        twist_msg.linear.z = velocity_list[2]
        twist_msg.angular.x = angular_velocity_list[0]
        twist_msg.angular.y = angular_velocity_list[1]
        twist_msg.angular.z = angular_velocity_list[2]

        twist_pub_.publish(twist_msg)

        #just make sure the node doesn't shut down
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('platform_manager', anonymous=True)
    try:
        p_manager = PlatformManager()
    except:
        rospy.ROSInterruptException
    pass
