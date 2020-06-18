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
from nav_msgs.msg import Odometry
from ublox.msg import PosVelEcef


class SimManager():

    def __init__(self):

        self.plt_pos = np.zeros(3)
        self.plt_prev_pos = np.zeros(3)
        self.plt_prev_time = 0.0
        self.plt_vel = np.zeros(3)
        self.drone_pos = np.zeros(3)
        self.lla = np.zeros(3)
        self.relPos = RelPos()
        self.rover_PosVelEcef = PosVelEcef()
        self.base_PosVelEcef = PosVelEcef()

        # Set Up Publishers and Subscribers
        self.platform_virtual_odom_pub_ = rospy.Publisher('platform_virtual_odometry', Odometry, queue_size=5, latch=True)
        self.relpos_pub_ = rospy.Publisher('rover_relpos', RelPos, queue_size=5, latch=True)  
        self.rover_PosVelEcef_pub_ = rospy.Publisher('rover_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        self.base_PosVelEcef_pub_ = rospy.Publisher('base_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
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

        
        self.plt_vel = self.calc_base_velocity(msg.header.stamp)

        # #Todo: switch message type to be the same as coming from UBLOX
        # x = Odometry()
        # x.header.stamp = msg.header.stamp
        # x.pose.pose.position.x = self.plt_pos[0]
        # x.pose.pose.position.y = -self.plt_pos[1]
        # x.pose.pose.position.z = self.plt_pos[2]
        # x.twist.twist.linear.x = self.plt_vel[0]
        # x.twist.twist.linear.y = -self.plt_vel[1] #should this negative be kept?
        # x.twist.twist.linear.z = self.plt_vel[2]
        # self.platform_virtual_odom_pub_.publish(x)

    def droneOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.drone_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])

    def gnssCallback(self, msg):
        # Callback is just used to publish all gps data at the same rate
        self.publish_virtual_rover_PosVelEcef(msg)
        self.publish_virtual_rover_relPos(msg.header)
        self.publish_virtual_base_PosVelEcef()

    def gnssRawCallback(self, msg):
        self.lla[0] = msg.lon
        self.lla[1] = msg.lat
        self.lla[2] = msg.height

    def publish_virtual_rover_PosVelEcef(self, msg):
        
        self.rover_PosVelEcef.header = msg.header
        self.rover_PosVelEcef.fix = msg.fix
        self.rover_PosVelEcef.lla = self.lla
        self.rover_PosVelEcef.position = msg.position
        self.rover_PosVelEcef.horizontal_accuracy = msg.horizontal_accuracy
        self.rover_PosVelEcef.vertical_accuracy = msg.vertical_accuracy
        self.rover_PosVelEcef.velocity = msg.velocity
        self.rover_PosVelEcef.speed_accuracy = msg.speed_accuracy

        self.rover_PosVelEcef_pub_.publish(self.rover_PosVelEcef)


    def publish_virtual_rover_relPos(self, gnss_header):
        relPos_array = self.drone_pos - self.plt_pos

        self.relPos.header = gnss_header
        self.relPos.relPosNED[0] = relPos_array[0]
        self.relPos.relPosNED[1] = relPos_array[1]
        self.relPos.relPosNED[2] = relPos_array[2]

        self.relpos_pub_.publish(self.relPos)

    def publish_virtual_base_PosVelEcef(self):

        self.base_PosVelEcef.velocity = self.plt_vel

        self.base_PosVelEcef_pub_.publish(self.base_PosVelEcef)

    def calc_base_velocity(self, stamp_msg):
        current_time = stamp_msg.secs+stamp_msg.nsecs*1e-9
        dt = current_time - self.plt_prev_time
        self.plt_prev_time = current_time

        #numerical differentiation to get velocity
        velocity = (self.plt_pos - self.plt_prev_pos)/dt
        self.plt_prev_pos = self.plt_pos

        return velocity


if __name__ == '__main__':
    rospy.init_node('sim_manager', anonymous=True)
    try:
        sim_manager = SimManager()
    except:
        rospy.ROSInterruptException
    pass
