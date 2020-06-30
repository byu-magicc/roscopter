#!/usr/bin/env python3

import numpy as np
import rospy

from IPython.core.debugger import set_trace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ublox.msg import PosVelEcef
from ublox.msg import RelPos
from rosflight_msgs.msg import GNSS 

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
        
        # self.ref_lla = np.array([40.267320, -111.635629, 1387.0])
        self.ref_lla = np.array([40.267320, -111.635629, 1387.0])
        self.A = 6378137.0       # WGS-84 Earth semimajor axis (m)
        self.B = 6356752.314245  # Derived Earth semiminor axis (m)
        self.F = (self.A - self.B)/self.A     # Ellipsoid Flatness
        self.E2 = self.F * (2.0 - self.F);    # Square of Eccentricity
        self.ecef = np.zeros(3)
        # self.ref_ecef = self.lla2ecef(self.ref_lla)
        # print('ref ecef = ', self.ref_ecef)
        # ecef2 = self.ned2ecef(np.array([3.2, 1.2, -5.4]), self.ref_lla)

        # self.rover_ned = np.zeros(3)
        # self.rover_prev_ned = np.zeros(3)
        # self.rover_prev_time = 0.0
        # self.base_ned = np.zeros(3)
        # self.base_prev_ned = np.zeros(3)
        # self.base_prev_time = 0.0
        self.rover_PosVelEcef = PosVelEcef()
        # self.rover_PosVelEcef_truth = PosVelEcef()
        # self.base_PosVelEcef = PosVelEcef()
        # self.rover_relPos = RelPos()

        # #for debugging
        # self.rover_gazebo_PosVelEcef = PosVelEcef()
        
        # self.rover_virtual_relpos_pub_ = rospy.Publisher('rover_relpos', RelPos, queue_size=5, latch=True)
        self.rover_virtual_PosVelEcef_pub_ = rospy.Publisher('rover_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        # self.base_virtual_PosVelEcef_pub_ = rospy.Publisher('base_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        # self.rover_mocap_ned_sub_ = rospy.Subscriber('rover_mocap', PoseStamped, self.roverMocapNedCallback, queue_size=5)
        # self.base_mocap_ned_sub_ = rospy.Subscriber('base_mocap', PoseStamped, self.baseMocapNedCallback, queue_size=5)
        # self.rover_gazebo_ecef_sub_ = rospy.Subscriber('/multirotor/gps/data', GNSS, self.roverGazeboEcefCallback, queue_size=5)
        # self.ublox_rate_timer_ = rospy.Timer(rospy.Duration(1.0/ublox_frequency), self.ubloxRateCallback)
        # self.rover_truth_sub_ = rospy.Subscriber('/multirotor/ground_truth/odometry/NED', Odometry, self.truthCallback, queue_size=5)
        self.rover_gnss_sub_ = rospy.Subscriber("gnss", GNSS, self.roverGnssCallback, queue_size=5)

        # while not rospy.is_shutdown():
        #     # wait for new messages and call the callback when they arrive
        #     rospy.spin()

    # def roverGazeboEcefCallback(self, msg):
    #     self.rover_gazebo_PosVelEcef.header = msg.header
    #     self.rover_gazebo_PosVelEcef.fix = msg.fix
    #     self.rover_gazebo_PosVelEcef.lla = [0.0, 0.0, 0.0]
    #     self.rover_gazebo_PosVelEcef.position = msg.position
    #     self.rover_gazebo_PosVelEcef.horizontal_accuracy = msg.horizontal_accuracy
    #     self.rover_gazebo_PosVelEcef.vertical_accuracy = msg.vertical_accuracy
    #     self.rover_gazebo_PosVelEcef.velocity = msg.velocity
    #     self.rover_gazebo_PosVelEcef.speed_accuracy = msg.speed_accuracy

    # def roverMocapNedCallback(self, msg):
        
    #     #TODO incoming rover messages are received much more frequently than base messages.  Why?
    #     self.rover_ned = np.array([msg.pose.position.x,
    #                                msg.pose.position.y,
    #                                msg.pose.position.z])
    #     # print('rover_ned = ', self.rover_ned)

    # def truthCallback(self, msg):

    #     truth_ned = msg.pose.pose.position
    #     # print('truth_ned = ', truth_ned)



    # def baseMocapNedCallback(self, msg):
        
    #     self.base_ned = np.array([msg.pose.position.x,
    #                               msg.pose.position.y,
    #                               msg.pose.position.z])


    # def ubloxRateCallback(self, event):

    #     # self.publish_rover_virtual_PosVelEcef()
    #     self.publish_rover_virtual_relPos()
    #     self.publish_base_virtual_PosVelEcef()


    # def publish_rover_virtual_PosVelEcef(self):

    #     print('in publish rover PosVelEcef')
    #     self.rover_PosVelEcef.header.stamp = rospy.Time.now()

    #     rover_ned_noise_n = self.add_noise(self.rover_ned[0], 0.0)#self.global_horizontal_accuracy)
    #     rover_ned_noise_e = self.add_noise(self.rover_ned[1], 0.0)#self.global_horizontal_accuracy)
    #     rover_ned_noise_d = self.add_noise(self.rover_ned[2], 0.0)#self.global_vertical_accuracy)
    #     rover_ned_noise = np.array([rover_ned_noise_n, rover_ned_noise_e, rover_ned_noise_d])

    #     current_time = self.rover_PosVelEcef.header.stamp.secs+self.rover_PosVelEcef.header.stamp.nsecs*1e-9
    #     dt = current_time - self.rover_prev_time
    #     self.rover_prev_time = current_time

    #     rover_vel = (self.rover_ned - self.rover_prev_ned)/dt

    #     rover_vel_noise_n = self.add_noise(rover_vel[0], self.global_speed_accuracy)
    #     rover_vel_noise_e = self.add_noise(rover_vel[1], self.global_speed_accuracy)
    #     rover_vel_noise_d = self.add_noise(rover_vel[2], self.global_speed_accuracy)
    #     rover_vel_noise = np.array([rover_vel_noise_n, rover_vel_noise_e, rover_vel_noise_d])
    #     rover_vel_ecef = self.ned2ecef(rover_vel_noise, self.ref_lla)

    #     self.rover_PosVelEcef.fix = 3
    #     # self.rover_PosVelEcef.lla = self.rover_lla  #lla is not currently being used

    #     # print('rover_ned_noise = ', rover_ned_noise)
    #     delta_ecef = self.ned2ecef(rover_ned_noise, self.ref_lla)
    #     # print('delta error = ', self.rover_gazebo_PosVelEcef.position - self.ref_ecef - delta_ecef)

    #     print('Before rover PosVelEcef')

    #     virtual_pos = self.ref_ecef + delta_ecef
    #     # print('virtual_pos = ', virtual_pos)
    #     # print('pos dif = ', virtual_pos - self.rover_gazebo_PosVelEcef.position)
    #     # print('rover ned = ', self.rover_ned)
    #     # print('delta_ecef = ', delta_ecef)
    #     # print('vel dif = ', rover_vel_ecef - self.rover_gazebo_PosVelEcef.velocity)
    #     self.rover_PosVelEcef.position = self.ref_ecef + delta_ecef
    #     self.rover_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
    #     self.rover_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
    #     self.rover_PosVelEcef.velocity = self.rover_gazebo_PosVelEcef.velocity #rover_vel
    #     self.rover_PosVelEcef.speed_accuracy = self.global_speed_accuracy

    #     print('After rover_PosVelEcef')

    #     self.rover_prev_ned = self.rover_ned

    #     # self.rover_virtual_PosVelEcef_pub_.publish(self.rover_PosVelEcef)

    def roverGnssCallback(self, msg):
        
        self.rover_PosVelEcef.header = msg.header
        self.rover_PosVelEcef.fix = msg.fix
        # self.rover_PosVelEcef.lla = self.rover_lla  #lla is not currently being used
        self.rover_PosVelEcef.position = msg.position
        self.rover_PosVelEcef.horizontal_accuracy = msg.horizontal_accuracy
        self.rover_PosVelEcef.vertical_accuracy = msg.vertical_accuracy
        self.rover_PosVelEcef.velocity = msg.velocity
        self.rover_PosVelEcef.speed_accuracy = msg.speed_accuracy

        # print('ecef error = ', np.array(self.rover_PosVelEcef.position)-np.array(self.rover_PosVelEcef_truth.position))

        self.rover_virtual_PosVelEcef_pub_.publish(self.rover_PosVelEcef)


    # def publish_rover_virtual_relPos(self):

    #     relPos_array = self.rover_ned - self.base_ned

    #     self.rover_relPos.header.stamp = rospy.Time.now()
    #     self.rover_relPos.relPosNED[0] = self.add_noise(relPos_array[0], self.relative_horizontal_accuracy)
    #     self.rover_relPos.relPosNED[1] = self.add_noise(relPos_array[1], self.relative_horizontal_accuracy)
    #     self.rover_relPos.relPosNED[2] = self.add_noise(relPos_array[2], self.relative_vertical_accuracy)

    #     self.rover_virtual_relpos_pub_.publish(self.rover_relPos)

    # def publish_base_virtual_PosVelEcef(self):

    #     self.base_PosVelEcef.header.stamp = rospy.Time.now()

    #     base_ned_noise_n = self.add_noise(self.base_ned[0], self.global_horizontal_accuracy)
    #     base_ned_noise_e = self.add_noise(self.base_ned[1], self.global_horizontal_accuracy)
    #     base_ned_noise_d = self.add_noise(self.base_ned[2], self.global_vertical_accuracy)
    #     base_ned_noise = np.array([base_ned_noise_n, base_ned_noise_e, base_ned_noise_d])

    #     current_time = self.base_PosVelEcef.header.stamp.secs+self.base_PosVelEcef.header.stamp.nsecs*1e-9
    #     dt = current_time - self.base_prev_time
    #     self.base_prev_time = current_time

    #     base_vel = (self.base_ned - self.base_prev_ned)/dt

    #     base_vel_noise_n = self.add_noise(base_vel[0], self.global_speed_accuracy)
    #     base_vel_noise_e = self.add_noise(base_vel[1], self.global_speed_accuracy)
    #     base_vel_noise_d = self.add_noise(base_vel[2], self.global_speed_accuracy)
    #     base_vel_noise = np.array([base_vel_noise_n, base_vel_noise_e, base_vel_noise_d])

    #     self.base_PosVelEcef.fix = 3
    #     # self.base_PosVelEcef.lla = self.base_lla  #lla is not currently being used

    #     delta_ecef = self.ned2ecef(base_ned_noise, self.ref_lla)

    #     self.base_PosVelEcef.position = self.rover_gazebo_PosVelEcef.position#self.ref_ecef + delta_ecef
    #     self.base_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
    #     self.base_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
    #     self.base_PosVelEcef.velocity = base_vel
    #     self.base_PosVelEcef.speed_accuracy = self.global_speed_accuracy

    #     self.base_prev_ned = self.base_ned

    #     self.base_virtual_PosVelEcef_pub_.publish(self.base_PosVelEcef)
    
    # def add_noise(self, value, std_dev):

    #     # print('value = ', value)
    #     value_w_noise = np.random.normal(value, std_dev, value.shape)
    #     # print('value with noise = ', value_w_noise)
    #     return value_w_noise

    # def lla2ecef(self, lla):

    #     lat = lla[0]*np.pi/180
    #     lon = lla[1]*np.pi/180
    #     alt = lla[2]
    #     sinp = np.sin(lat)
    #     cosp = np.cos(lat)
    #     sinl = np.sin(lon)
    #     cosl = np.cos(lon)
    #     e2 = self.E2
    #     v = self.A/np.sqrt(1.0-e2*sinp*sinp)

    #     self.ecef[0]=(v+alt)*cosp*cosl
    #     self.ecef[1]=(v+alt)*cosp*sinl
    #     self.ecef[2]=(v*(1.0-e2)+lla[2])*sinp

    #     return self.ecef

    # def ned2ecef(self, ned, lla):
        
    #     lat = lla[0]*np.pi/180
    #     lon = lla[1]*np.pi/180
    #     alt = lla[2]

    #     ecef = self.Ry(90.0)@self.Rx(-lon)@self.Ry(-lat)@ned

    #     return ecef

    # def Rx(self, theta):
        
    #     theta = theta*np.pi/180.0
    #     st = np.sin(theta)
    #     ct = np.cos(theta)
    #     rotx = np.array([[1.0, 0.0, 0.0], \
    #                      [0.0, ct, st], \
    #                      [0.0, -st, ct]])

    #     return rotx

    # def Ry(self, theta):
        
    #     theta = theta*np.pi/180.0
    #     st = np.sin(theta)
    #     ct = np.cos(theta)
    #     roty = np.array([[ct, 0.0, -st], \
    #                     [0.0, 1.0, 0.0], \
    #                     [st, 0.0, ct]])

    #     return roty

    # def Rz(self, theta):

    #     theta = theta*np.pi/180.0
    #     st = np.sin(theta)
    #     ct = np.cos(theta)
    #     rotz = np.array([[ct, st, 0.0], \
    #                      [-st, ct, 0.0], \
    #                      [0.0, 0.0, 1.0]])

    #     return rotz


if __name__ == '__main__':

    rospy.init_node('mocap2ublox', anonymous=True)
    try:
        mocap2ublox = Mocap2Ublox()
    except:
        rospy.RosInterruptException
    pass
