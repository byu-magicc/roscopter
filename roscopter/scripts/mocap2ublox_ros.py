#!/usr/bin/env python3

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from ublox.msg import PosVelEcef
from ublox.msg import RelPos

class Mocap2Ublox():

    def __init__(self):

        #parameters
        ublox_frequency = 5.0 #hz
        self.global_horizontal_accuracy = 0.4
        self.global_vertical_accuracy = 0.6
        self.global_speed_accuracy = 0.4
        self.relative_horizontal_accuracy = 0.02
        self.relative_vertical_accuracy = 0.06
        self.relative_speed_accuracy = 0.02

        self.rover_ned = np.zeros(3)
        self.rover_prev_ned = np.zeros(3)
        self.rover_prev_time = 0.0
        self.base_ned = np.zeros(3)
        self.base_prev_ned = np.zeros(3)
        self.base_prev_time = 0.0
        self.rover_PosVelEcef = PosVelEcef()
        self.base_PosVelEcef = PosVelEcef()
        self.rover_relPos = RelPos()
        
        self.rover_virtual_relpos_pub_ = rospy.Publisher('rover_relpos', RelPos, queue_size=5, latch=True)  
        self.rover_virtual_PosVelEcef_pub_ = rospy.Publisher('rover_PosVelEcef', PosVelEcef, queue_size=5, latch=True)        self.rover_mocap_ned_sub_ = rospy.Subscriber('rover_mocap', PoseStamped, self.roverMocapNedCallback, queue_size=5)
        self.base_virtual_PosVelEcef_pub_ = rospy.Publisher('base_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        self.base_mocap_ned_sub_ = rospy.Subscriber('base_mocap', PoseStamped, self.baseMocapNedCallback, queue_size=5)
        self.ublox_rate_timer_ = rospy.Timer(rospy.Duration(1.0/ublox_frequency), self.ubloxRateCallback)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def roverMocapNedCallback(self, msg):
        self.rover_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])


    def baseMocapNedCallback(self, msg):
        self.base_ned = np.array([msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z])


    def ubloxRateCallback(self, event):

        self.publish_rover_virtual_PosVelEcef()
        self.publish_rover_virtual_relPos()
        self.publish_base_virtual_PosVelEcef()


    def publish_rover_virtual_PosVelEcef(self):

        self.rover_PosVelEcef.header.stamp = rospy.Time.now()

        rover_ned_noise_n = self.add_noise(self.rover_ned[0], self.global_horizontal_accuracy)
        rover_ned_noise_e = self.add_noise(self.rover_ned[1], self.global_horizontal_accuracy)
        rover_ned_noise_d = self.add_noise(self.rover_ned[2], self.global_vertical_accuracy)
        rover_ned_noise = np.array([rover_ned_noise_n, rover_ned_noise_e, rover_ned_noise_d])

        current_time = self.rover_PosVelEcef.header.stamp.secs+self.rover_PosVelEcef.header.stamp.nsecs*1e-9
        dt = current_time - self.rover_prev_time
        self.rover_prev_time = current_time

        rover_vel = (self.rover_ned - self.rover_prev_ned)/dt

        rover_vel_noise_n = self.add_noise(rover_vel[0], self.global_speed_accuracy)
        rover_vel_noise_e = self.add_noise(rover_vel[1], self.global_speed_accuracy)
        rover_vel_noise_d = self.add_noise(rover_vel[2], self.global_speed_accuracy)
        rover_vel_noise = np.array([rover_vel_noise_n, rover_vel_noise_e, rover_vel_noise_d])

        self.rover_PosVelEcef.fix = 3.0
        # self.rover_PosVelEcef.lla = self.rover_lla  #lla is not currently being used
        self.rover_PosVelEcef.position = rover_ned_noise
        self.rover_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
        self.rover_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
        self.rover_PosVelEcef.velocity = rover_vel
        self.rover_PosVelEcef.speed_accuracy = self.global_speed_accuracy

        self.rover_prev_ned = self.rover_ned

        self.rover_virtual_PosVelEcef_pub_.publish(self.rover_PosVelEcef)


    def publish_rover_virtual_relPos(self):
        relPos_array = self.rover_ned - self.base_ned

        self.rover_relPos.header.stamp = rospy.Time.now()
        self.rover_relPos.relPosNED[0] = self.add_noise(relPos_array[0], self.relative_horizontal_accuracy)
        self.rover_relPos.relPosNED[1] = self.add_noise(relPos_array[1], self.relative_horizontal_accuracy)
        self.rover_relPos.relPosNED[2] = self.add_noise(relPos_array[2], self.relative_vertical_accuracy)

        self.rover_virtual_relpos_pub_.publish(self.rover_relPos)

    def publish_base_virtual_PosVelEcef(self):

        self.base_PosVelEcef.header.stamp = rospy.Time.now()

        base_ned_noise_n = self.add_noise(self.base_ned[0], self.global_horizontal_accuracy)
        base_ned_noise_e = self.add_noise(self.base_ned[1], self.global_horizontal_accuracy)
        base_ned_noise_d = self.add_noise(self.base_ned[2], self.global_vertical_accuracy)
        base_ned_noise = np.array([base_ned_noise_n, base_ned_noise_e, base_ned_noise_d])

        current_time = self.base_PosVelEcef.header.stamp.secs+self.base_PosVelEcef.header.stamp.nsecs*1e-9
        dt = current_time - self.base_prev_time
        self.base_prev_time = current_time

        base_vel = (self.base_ned - self.base_prev_ned)/dt

        base_vel_noise_n = self.add_noise(base_vel[0], self.global_speed_accuracy)
        base_vel_noise_e = self.add_noise(base_vel[1], self.global_speed_accuracy)
        base_vel_noise_d = self.add_noise(base_vel[2], self.global_speed_accuracy)
        base_vel_noise = np.array([base_vel_noise_n, base_vel_noise_e, base_vel_noise_d])

        self.base_PosVelEcef.fix = 3.0
        # self.base_PosVelEcef.lla = self.base_lla  #lla is not currently being used
        self.base_PosVelEcef.position = base_ned_noise
        self.base_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
        self.base_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
        self.base_PosVelEcef.velocity = base_vel
        self.base_PosVelEcef.speed_accuracy = self.global_speed_accuracy

        self.base_prev_ned = self.base_ned

        self.base_virtual_PosVelEcef_pub_.publish(self.base_PosVelEcef)
    
    def add_noise(self, value, std_dev):
        # print('value = ', value)
        value_w_noise = np.random.normal(value, std_dev, value.shape)
        # print('value with noise = ', value_w_noise)
        return value_w_noise


if __name__ == '__main__':

    rospy.init_node('mocap2ublox', anonymous=True)
    try:
        mocap2ublox = Mocap2Ublox()
    except:
        rospy.RosInterruptException
    pass