#!/usr/bin/env python3
import numpy as np
import rospy

class LQR_Controller():

    def __init__(self):
        self.equilibrium_throttle = 0.65
        self.gravity = 9.8
        self.mass = 3.69
        self.position = np.array([[0],[0],[0]])
        self.velocity = np.array([[0],[0],[0]])
        self.heading = 0
        self.attitude_in_SO3 = np.eye(3)
        self.desired_position = np.array([[0],[0],[0]])
        self.desired_velocity = np.array([[0],[0],[0]])
        self.desired_heading = 0

    def updateState(self, position, velocity, attitude_as_quaternion, time_step):
        self.position = position
        self.velocity = velocity
        self.attitude_in_SO3 = self.quaternionToSO3(attitude_as_quaternion)
        self.heading = self.getCurrentHeading()

    def updateDesiredState(self, desired_position, desired_velocity, desired_heading):
        self.desired_position = desired_position
        self.desired_velocity = desired_velocity
        self.desired_heading = desired_heading
        self.desired_heading_rate = desired_heading_rate

    def computeRosflightCommand(self):
        #Implement Code Here

        roll_rate = body_angular_rate_commands.item(0)
        pitch_rate = body_angular_rate_commands.item(1)
        yaw_rate = body_angular_rate_commands.item(2)
        rosflight_command = np.array([roll_rate, pitch_rate, yaw_rate , throttle])
        return rosflight_command

    def veeOperator(self , skewSymmetricMatrix):
        return np.array([[skewSymmetricMatrix[2,1]] , [skewSymmetricMatrix[0,2]] , [skewSymmetricMatrix[1,0]]])

    def finiteDifferencing(self, value, new_value, time_step):
        return (new_value - value)/time_step

    def quaternionToSO3(self,quaternion):
        q = quaternion
        r00 = 2 * (q.item(0)*q.item(0) + q.item(1)*q.item(1)) - 1
        r01 = 2 * (q.item(1)*q.item(2) - q.item(0)*q.item(3))
        r02 = 2 * (q.item(1)*q.item(3) + q.item(0)*q.item(2))
        r10 = 2 * (q.item(1)*q.item(2) + q.item(0)*q.item(3))
        r11 = 2 * (q.item(0)*q.item(0) + q.item(2)*q.item(2)) - 1
        r12 = 2 * (q.item(2)*q.item(3) - q.item(0)*q.item(1))
        r20 = 2 * (q.item(1)*q.item(3) - q.item(0)*q.item(2))
        r21 = 2 * (q.item(2)*q.item(3) + q.item(0)*q.item(1))
        r22 = 2 * (q.item(0)*q.item(0) + q.item(3)*q.item(3)) - 1
        matrix_SO3 = np.array([[r00, r01, r02],
                                        [r10, r11, r12],
                                        [r20, r21, r22]])
        return matrix_SO3

    def getCurrentHeading(self):
        north = np.array([[1],[0],[0]])
        inertialXDir =  np.dot(self.attitude_in_SO3,north)
        headingAngle = np.abs(np.arccos(np.dot(north.flatten(), inertialXDir.flatten()) / ( np.linalg.norm(north) * np.linalg.norm(inertialXDir) ))) * np.sign(inertialXDir.item(1))
        return headingAngle

    def getCurrentTrajectoryState(self):
        state = np.array([])
        x_position = self.position[0]
        y_position = self.position[1]
        z_position = self.position[2]
        x_velocity = self.velocity[0]
        y_velocity = self.velocity[1]
        z_velocity = self.velocity[2]
        x_acceleration = self.acceleration[0]
        y_acceleration = self.acceleration[1] 
        z_acceleration = self.acceleration[2] 
        x_jerk = self.jerk[0]
        y_jerk = self.jerk[1]
        z_jerk = self.jerk[2]
        heading = self.heading
        heading_rate = self.heading_rate
        state = np.array([x_position, y_position, z_position , x_velocity , y_velocity, z_velocity,
                x_acceleration, y_acceleration, z_acceleration, x_jerk, y_jerk, z_jerk, 
                heading, heading_rate])
        return state