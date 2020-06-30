

import numpy as np


class Mocap2Ublox():
    
    def __init__(self, Ts, gha, gva, gsa, rha, rva, rsa, rl, srp, srv, srr, sbp, sbv, A, B, F, E2, re):

        self.Ts = Ts
        self.global_horizontal_accuracy = gha
        self.global_vertical_accuracy = gva
        self.global_speed_accuracy = gsa
        self.relative_horizontal_accuracy = rha
        self.relative_vertical_accuracy = rva
        self.relative_speed_accuracy = rsa
        self.ref_lla = rl
        self.sigma_rover_pos = srp 
        self.sigma_rover_vel = srv
        self.sigma_rover_relpos = srr
        self.sigma_base_pos = sbp
        self.sigma_base_vel = sbv
        self.A = A
        self.B = B
        self.F = F     
        self.E2 = E2    
        self.ref_ecef = re

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

        #Outputs
        self.rover_virtual_pos_ecef = np.zeros(3)
        self.rover_virtual_vel_ecef = np.zeros(3)

    def update_rover_virtual_PosVelEcef(self, dt):

        #calculate virtual position in ecef frame with noise
        rover_ned_noise_n = self.add_noise(self.rover_ned[0], self.global_horizontal_accuracy)
        rover_ned_noise_e = self.add_noise(self.rover_ned[1], self.global_horizontal_accuracy)
        rover_ned_noise_d = self.add_noise(self.rover_ned[2], self.global_vertical_accuracy)
        rover_ned_noise = np.array([rover_ned_noise_n, rover_ned_noise_e, rover_ned_noise_d])
        self.rover_ned_lpf = self.lpf(rover_ned_noise, self.rover_ned_lpf, self.Ts, self.sigma_rover_pos)
        virtual_delta_ecef = self.ned2ecef(self.rover_ned_lpf, self.ref_lla)
        self.rover_virtual_pos_ecef = self.ref_ecef + virtual_delta_ecef

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
        self.rover_virtual_vel_ecef = self.ned2ecef(rover_vel_noise, self.ref_lla)

        #update histories
        self.rover_ned_prev = self.rover_ned
        self.rover_vel_prev = rover_vel


    def add_noise(self, value, std_dev):

        value_w_noise = np.random.normal(value, std_dev, value.shape)
        return value_w_noise


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
        