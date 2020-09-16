#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from roscopter_msgs.msg import Command
from roscopter_msgs.msg import Control_Performance
from performance import Performance

class PerformanceROS():

    def __init__(self):

        riseTimeCutoffPercentage = rospy.get_param('~riseTimeCutoffPercentage', 90.0)
        settleTimeEnvelopePercentage = rospy.get_param('~settleTimeEnvelopePercentage', 10.0)

        self.performance = Performance(riseTimeCutoffPercentage, settleTimeEnvelopePercentage)

        self.performance_message = Control_Performance()

        # Publishers
        self.body_frame_preformance_pub_ = rospy.Publisher('controller_performance', Control_Performance, queue_size=5, latch=True)
        
        # Subscribers
        self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odomCallback, queue_size=5)
        self.high_level_command_sub = rospy.Subscriber('high_level_command', Command, self.highLevelCommandCallback, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def odomCallback(self, msg):
        
        self.performance.north.state = msg.pose.pose.position.x
        self.performance.east.state = msg.pose.pose.position.y
        self.performance.down.state = msg.pose.pose.position.z

        quat = [msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z]
        euler = self.get_euler(quat)
        self.performance.yaw.state = euler[2]

        self.performance.odomTime = msg.header.stamp.secs + msg.header.stamp.nsecs*1E-9
    
    def highLevelCommandCallback(self, msg):

        self.publish_performance()
        
        self.performance.north.highLevelCommand = msg.cmd1
        self.performance.east.highLevelCommand = msg.cmd2
        self.performance.down.highLevelCommand = msg.cmd3
        self.performance.yaw.highLevelCommand = msg.cmd4

        self.performance.receivedCommandTime = msg.stamp.secs + msg.stamp.nsecs*1E-9

        self.performance.update_performance_measures_for_all_states()
        self.performance.update_command_for_all_states()

    def publish_performance(self):

        #TODO these need to be rotated to the body frame
        self.performance_message.north_body_frame_rise_time = self.performance.north.rise_time
        self.performance_message.north_body_frame_settling_time = self.performance.north.settle_time
        self.performance_message.north_body_frame_percent_overshoot = self.performance.north.percent_overshoot
        self.performance_message.east_body_frame_rise_time = self.performance.east.rise_time
        self.performance_message.east_body_frame_settling_time = self.performance.east.settle_time
        self.performance_message.east_body_frame_percent_overshoot = self.performance.east.percent_overshoot
        self.performance_message.down_body_frame_rise_time = self.performance.down.rise_time
        self.performance_message.down_body_frame_settling_time = self.performance.down.settle_time
        self.performance_message.down_body_frame_percent_overshoot = self.performance.down.percent_overshoot   
        self.performance_message.yaw_body_frame_rise_time = self.performance.yaw.rise_time
        self.performance_message.yaw_body_frame_settling_time = self.performance.yaw.settle_time
        self.performance_message.yaw_body_frame_settling_time = self.performance.yaw.percent_overshoot 

        self.body_frame_preformance_pub_.publish(self.performance_message)

    #this function works in ipython
    def get_euler(self, quat):  

        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]                     
        
        euler = np.array([np.arctan2(2.0*(qw*qx+qy*qz),1.0-2.0*(qx**2+qy**2)),
                          np.arcsin(2.0*(qw*qy-qz*qx)),
                          np.arctan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy**2+qz**2))])                                                       
        
        return euler      

if __name__ == '__main__':
    rospy.init_node('performance_ros', anonymous=True)
    try:
        performance_ros = PerformanceROS()
    except:
        rospy.ROSInterruptException
    pass