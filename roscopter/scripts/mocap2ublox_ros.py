#!/usr/bin/env python

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from ublox.msg import PosVelEcef
from ublox.msg import RelPos
from rosflight_msgs.msg import GNSS

from mocap2ublox import Mocap2Ublox


class Mocap2UbloxROS():

    def __init__(self):

        self.load_set_parameters()

        self.m2u = Mocap2Ublox(self.Ts, self.global_horizontal_accuracy, \
            self.global_vertical_accuracy, self.global_speed_accuracy, \
            self.relative_horizontal_accuracy, self.relative_vertical_accuracy, \
            self.relative_speed_accuracy, self.ref_lla, self.sigma_rover_pos, \
            self.sigma_rover_vel, self.sigma_rover_relpos, self.sigma_base_pos, \
            self.sigma_base_vel, self.A, self.B, self.F, self.E2, self.ref_ecef)

        # Publishers
        self.rover_virtual_relpos_pub_ = rospy.Publisher('rover_relpos', RelPos, queue_size=5, latch=True)
        self.rover_virtual_PosVelEcef_pub_ = rospy.Publisher('rover_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        self.base_virtual_PosVelEcef_pub_ = rospy.Publisher('base_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        
        # Subscribers
        self.rover_mocap_ned_sub_ = rospy.Subscriber('rover_mocap', PoseStamped, self.roverMocapNedCallback, queue_size=5)
        self.base_mocap_ned_sub_ = rospy.Subscriber('base_mocap', PoseStamped, self.baseMocapNedCallback, queue_size=5)
        
        # Timer
        self.ublox_rate_timer_ = rospy.Timer(rospy.Duration(self.Ts), self.ubloxRateCallback)


        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def roverMocapNedCallback(self, msg):
        
        self.m2u.rover_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])
        self.rover_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])


    def baseMocapNedCallback(self, msg):

        self.base_ned = np.array([msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z])

    
    def ubloxRateCallback(self, event):
        
        #publishes all the messages together like ublox would
        
        #TODO use event to get time
        #get time
        time_stamp = rospy.Time.now()
        current_time = time_stamp.secs+time_stamp.nsecs*1e-9
        dt = current_time - self.rover_prev_time

        self.m2u.update_rover_virtual_PosVelEcef(dt)
        # m2u.update_rover_virtual_relPos()
        # m2u.update_base_virtual_PosVelEcef(dt)
        self.publish_rover_virtual_PosVelEcef(time_stamp)
        self.publish_rover_virtual_relPos()
        self.publish_base_virtual_PosVelEcef()


    def publish_rover_virtual_PosVelEcef(self, time_stamp):

        current_time = time_stamp.secs+time_stamp.nsecs*1e-9
        dt = current_time - self.rover_prev_time

        #calculate virtual position in ecef frame with noise
        rover_ned_noise_n = self.add_noise(self.rover_ned[0], self.global_horizontal_accuracy)
        rover_ned_noise_e = self.add_noise(self.rover_ned[1], self.global_horizontal_accuracy)
        rover_ned_noise_d = self.add_noise(self.rover_ned[2], self.global_vertical_accuracy)
        rover_ned_noise = np.array([rover_ned_noise_n, rover_ned_noise_e, rover_ned_noise_d])
        self.rover_ned_lpf = self.lpf(rover_ned_noise, self.rover_ned_lpf, self.Ts, self.sigma_rover_pos)
        virtual_delta_ecef = self.ned2ecef(self.rover_ned_lpf, self.ref_lla)
        virtual_pos = self.ref_ecef + virtual_delta_ecef

        #calculate virtual velocity in ecef frame with noise
        #make sure we do not divide by zero
        if dt != 0.0:
            rover_vel = (self.rover_ned - self.rover_ned_prev)/dt
        else:
            rover_vel = np.zeros(3)
        rover_vel_noise_n = self.add_noise(rover_vel[0], self.global_speed_accuracy)
        rover_vel_noise_e = self.add_noise(rover_vel[1], self.global_speed_accuracy)
        rover_vel_noise_d = self.add_noise(rover_vel[2], self.global_speed_accuracy)
        rover_vel_noise = np.array([rover_vel_noise_n, rover_vel_noise_e, rover_vel_noise_d])
        self.rover_vel_lpf = self.lpf(rover_vel_noise, self.rover_vel_lpf, self.Ts, self.sigma_rover_vel)
        rover_vel_ecef = self.ned2ecef(rover_vel_noise, self.ref_lla)

        #package message
        self.rover_PosVelEcef.header.stamp = time_stamp
        self.rover_PosVelEcef.fix = 3
        # # self.rover_PosVelEcef.lla = self.rover_lla  #lla is not currently being used            
        self.rover_PosVelEcef.position = virtual_pos #self.m2u.rover_virtual_pos_ecef
        self.rover_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
        self.rover_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
        self.rover_PosVelEcef.velocity = rover_vel_ecef #self.m2u.rover_virtual_vel_ecef
        self.rover_PosVelEcef.speed_accuracy = self.global_speed_accuracy

        # #update histories
        self.rover_prev_time = current_time
        self.rover_ned_prev = self.rover_ned
        self.rover_vel_prev = rover_vel

        self.rover_virtual_PosVelEcef_pub_.publish(self.rover_PosVelEcef)


    def publish_rover_virtual_relPos(self):

        #calculate virtual relative position of the rover with respect to the base in ned frame with noise.
        relpos_array = self.rover_ned - self.base_ned
        rover_relpos_noise_n = self.add_noise(relpos_array[0], self.relative_horizontal_accuracy)
        rover_relpos_noise_e = self.add_noise(relpos_array[1], self.relative_horizontal_accuracy)
        rover_relpos_noise_d = self.add_noise(relpos_array[2], self.relative_vertical_accuracy)
        rover_relpos_noise = np.array([rover_relpos_noise_n, rover_relpos_noise_e, rover_relpos_noise_d])
        self.rover_relpos_lpf = self.lpf(rover_relpos_noise, self.rover_relpos_lpf, self.Ts, self.sigma_rover_relpos)

        #package up the message
        self.rover_relPos.header.stamp = rospy.Time.now()
        self.rover_relPos.relPosNED[0] = self.rover_relpos_lpf[0]
        self.rover_relPos.relPosNED[1] = self.rover_relpos_lpf[1]
        self.rover_relPos.relPosNED[2] = self.rover_relpos_lpf[2]

        self.rover_virtual_relpos_pub_.publish(self.rover_relPos)

    def publish_base_virtual_PosVelEcef(self):

        #get time
        time_stamp = rospy.Time.now()
        current_time = time_stamp.secs+time_stamp.nsecs*1e-9
        current_time = self.base_PosVelEcef.header.stamp.secs+self.base_PosVelEcef.header.stamp.nsecs*1e-9
        dt = current_time - self.base_prev_time

        #calculate virtual position in ecef frame with noise
        base_ned_noise_n = self.add_noise(self.base_ned[0], self.global_horizontal_accuracy)
        base_ned_noise_e = self.add_noise(self.base_ned[1], self.global_horizontal_accuracy)
        base_ned_noise_d = self.add_noise(self.base_ned[2], self.global_vertical_accuracy)
        base_ned_noise = np.array([base_ned_noise_n, base_ned_noise_e, base_ned_noise_d])
        self.base_ned_lpf = self.lpf(base_ned_noise, self.base_ned_lpf, self.Ts, self.sigma_base_pos)
        virtual_delta_ecef = self.ned2ecef(self.base_ned_lpf, self.ref_lla)
        virtual_pos = self.ref_ecef + virtual_delta_ecef

        #calculate virtual velocity in ecef frame with noise
        #make sure we do not divide by zero
        if dt != 0.0:
            base_vel = (self.base_ned - self.base_ned_prev)/dt
        else:
            base_vel = np.zeros(3)
        base_vel_noise_n = self.add_noise(base_vel[0], self.global_speed_accuracy)
        base_vel_noise_e = self.add_noise(base_vel[1], self.global_speed_accuracy)
        base_vel_noise_d = self.add_noise(base_vel[2], self.global_speed_accuracy)
        base_vel_noise = np.array([base_vel_noise_n, base_vel_noise_e, base_vel_noise_d])
        self.base_vel_lpf = self.lpf(base_vel_noise, self.base_vel_lpf, self.Ts, self.sigma_base_vel)
        base_vel_ecef = self.ned2ecef(base_vel_noise, self.ref_lla)

        #package message
        self.base_PosVelEcef.header.stamp = time_stamp
        self.base_PosVelEcef.fix = 3
        # # self.base_PosVelEcef.lla = self.base_lla  #lla is not currently being used            
        self.base_PosVelEcef.position = virtual_pos
        self.base_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
        self.base_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
        self.base_PosVelEcef.velocity = base_vel_ecef
        self.base_PosVelEcef.speed_accuracy = self.global_speed_accuracy

        #update histories
        self.base_prev_time = current_time
        self.base_ned_prev = self.base_ned
        self.base_vel_prev = base_vel

        self.base_virtual_PosVelEcef_pub_.publish(self.base_PosVelEcef)


    def add_noise(self, value, std_dev):

        value_w_noise = np.random.normal(value, std_dev, value.shape)
        return value_w_noise


    def lla2ecef(self, lla):

        lat = lla[0]*np.pi/180
        lon = lla[1]*np.pi/180
        alt = lla[2]
        sinp = np.sin(lat)
        cosp = np.cos(lat)
        sinl = np.sin(lon)
        cosl = np.cos(lon)
        e2 = self.E2
        v = self.A/np.sqrt(1.0-e2*sinp*sinp)

        ecef = np.zeros(3)
        ecef[0]=(v+alt)*cosp*cosl
        ecef[1]=(v+alt)*cosp*sinl
        ecef[2]=(v*(1.0-e2)+lla[2])*sinp

        return ecef


    def ned2ecef(self, ned, lla):
        
        lat = lla[0]
        lon = lla[1]
        # alt = lla[2]

        #don't know why @ isn't working for matrix multiplication
        # ecef = Ry(90)Rx(-lon)Ry(lat)ned
        ecef = np.matmul(np.matmul(np.matmul(self.Ry(90.0),self.Rx(-lon)),self.Ry(lat)),ned)

        return ecef

    def Rx(self, theta):
        
        theta = theta*np.pi/180.0
        st = np.sin(theta)
        ct = np.cos(theta)
        rotx = np.array([[1.0, 0.0, 0.0], \
                        [0.0, ct, st], \
                        [0.0, -st, ct]])

        return rotx


    def Ry(self, theta):
        
        theta = theta*np.pi/180.0
        st = np.sin(theta)
        ct = np.cos(theta)
        roty = np.array([[ct, 0.0, -st], \
                        [0.0, 1.0, 0.0], \
                        [st, 0.0, ct]])

        return roty


    def Rz(self, theta):

        theta = theta*np.pi/180.0
        st = np.sin(theta)
        ct = np.cos(theta)
        rotz = np.array([[ct, st, 0.0], \
                        [-st, ct, 0.0], \
                        [0.0, 0.0, 1.0]])

        return rotz

    def lpf(self, xt, x_prev, dt, sigma):
        
        x_lpf = xt*dt/(sigma+dt) + x_prev*sigma/(sigma+dt)
        
        return x_lpf

    
    def load_set_parameters(self):

        ublox_frequency = rospy.get_param('~ublox_frequency', 5.0)
        self.Ts = 1.0/ublox_frequency
        self.global_horizontal_accuracy = rospy.get_param('~global_horizontal_accuracy', 0.4)
        self.global_vertical_accuracy = rospy.get_param('~global_vertical_accuracy', 0.6)
        self.global_speed_accuracy = rospy.get_param('~global_speed_accuracy', 0.4)
        self.relative_horizontal_accuracy = rospy.get_param('~relative_horizontal_accuracy', 0.02)
        self.relative_vertical_accuracy = rospy.get_param('~relative_vertical_accuracy', 0.06)
        self.relative_speed_accuracy = rospy.get_param('~relative_speed_accuracy', 0.02)
        ref_lla = rospy.get_param('~ref_lla', [40.267320, -111.635629, 1387.0])
        self.ref_lla = np.array(ref_lla)
        self.sigma_rover_pos = rospy.get_param('~sigma_rover_pos', 5.0) 
        self.sigma_rover_vel = rospy.get_param('~sigma_rover_vel', 5.0)
        self.sigma_rover_relpos = rospy.get_param('~sigma_rover_relpos', 0.0)
        self.sigma_base_pos = rospy.get_param('~sigma_base_pos', 0.0)
        self.sigma_base_vel = rospy.get_param('~sigma_base_vel', 0.0)
        self.A = rospy.get_param('~A', 6378137.0)
        self.B = rospy.get_param('~B', 6356752.314245)
    
        #calc ref_ecef from ref_lla
        self.F = (self.A - self.B)/self.A     # Ellipsoid Flatness
        self.E2 = self.F * (2.0 - self.F);    # Square of Eccentricity
        self.ref_ecef = self.lla2ecef(self.ref_lla)

        #message types
        self.rover_PosVelEcef = PosVelEcef()
        self.base_PosVelEcef = PosVelEcef()
        self.rover_relPos = RelPos()

        #other needed variables and arrays
        self.rover_ned = np.zeros(3)
        self.rover_ned_prev = np.zeros(3)
        self.rover_ned_lpf = np.zeros(3)
        self.rover_vel_lpf = np.zeros(3)
        self.rover_prev_time = 0.0
        self.rover_relpos_lpf = np.zeros(3)
        self.base_ned = np.zeros(3)
        self.base_ned_prev = np.zeros(3)
        self.base_ned_lpf = np.zeros(3)
        self.base_vel_lpf = np.zeros(3)
        self.base_prev_time = 0.0
        

if __name__ == '__main__':
    rospy.init_node('mocap2ublox_ros', anonymous=True)
    try:
        mocap2ublox_ros = Mocap2UbloxROS()
    except:
        rospy.ROSInterruptException
    pass
