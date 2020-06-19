#!/usr/bin/env python

import numpy as np
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from rosflight_msgs.msg import GNSS
from rosflight_msgs.msg import GNSSRaw
from ublox.msg import RelPos
from ublox.msg import PosVelEcef


class SimManager():

    def __init__(self):

        self.base_pos = np.zeros(3)
        self.base_prev_pos = np.zeros(3)
        self.base_prev_time = 0.0
        self.base_vel = np.zeros(3)
        self.rover_pos = np.zeros(3)
        self.rover_lla = np.zeros(3)
        self.rover_relPos = RelPos()
        self.rover_PosVelEcef = PosVelEcef()
        self.base_PosVelEcef = PosVelEcef()

        # Set Up Publishers and Subscribers
        self.rover_virtual_relpos_pub_ = rospy.Publisher('rover_relpos', RelPos, queue_size=5, latch=True)  
        self.rover_virtual_PosVelEcef_pub_ = rospy.Publisher('rover_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        self.base_virtual_PosVelEcef_pub_ = rospy.Publisher('base_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        self.base_odom_sub_ = rospy.Subscriber('platform_odom', Odometry, self.baseOdomCallback, queue_size=5)
        self.rover_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.roverOdomCallback, queue_size=5)
        self.rover_gnss_sub_ = rospy.Subscriber("gnss", GNSS, self.roverGnssCallback, queue_size=5)

        
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def baseOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.base_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])

        
        self.base_vel = self.calc_base_velocity(msg.header.stamp)

    def roverOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.rover_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])

    def roverGnssCallback(self, msg):
        # Callback is just used to publish all gps data at the same rate
        self.publish_rover_virtual_PosVelEcef(msg)
        self.publish_rover_virtual_relPos(msg.header)
        self.publish_base_virtual_PosVelEcef()

    def roverGnssRawCallback(self, msg):
        self.rover_lla[0] = msg.lon
        self.rover_lla[1] = msg.lat
        self.rover_lla[2] = msg.height

    def publish_rover_virtual_PosVelEcef(self, msg):
        
        self.rover_PosVelEcef.header = msg.header
        self.rover_PosVelEcef.fix = msg.fix
        # self.rover_PosVelEcef.lla = self.rover_lla  #lla is not currently being used
        self.rover_PosVelEcef.position = msg.position
        self.rover_PosVelEcef.horizontal_accuracy = msg.horizontal_accuracy
        self.rover_PosVelEcef.vertical_accuracy = msg.vertical_accuracy
        self.rover_PosVelEcef.velocity = msg.velocity
        self.rover_PosVelEcef.speed_accuracy = msg.speed_accuracy

        self.rover_virtual_PosVelEcef_pub_.publish(self.rover_PosVelEcef)


    def publish_rover_virtual_relPos(self, gnss_header):
        relPos_array = self.rover_pos - self.base_pos

        self.rover_relPos.header = gnss_header
        self.rover_relPos.relPosNED[0] = relPos_array[0]
        self.rover_relPos.relPosNED[1] = relPos_array[1]
        self.rover_relPos.relPosNED[2] = relPos_array[2]

        self.rover_virtual_relpos_pub_.publish(self.rover_relPos)

    def publish_base_virtual_PosVelEcef(self):

        self.base_PosVelEcef.velocity = self.base_vel

        self.base_virtual_PosVelEcef_pub_.publish(self.base_PosVelEcef)

    def calc_base_velocity(self, stamp_msg):
        current_time = stamp_msg.secs+stamp_msg.nsecs*1e-9
        dt = current_time - self.base_prev_time
        self.base_prev_time = current_time

        #numerical differentiation to get velocity
        velocity = (self.base_pos - self.base_prev_pos)/dt
        self.base_prev_pos = self.base_pos

        return velocity


if __name__ == '__main__':
    rospy.init_node('sim_manager', anonymous=True)
    try:
        sim_manager = SimManager()
    except:
        rospy.ROSInterruptException
    pass
