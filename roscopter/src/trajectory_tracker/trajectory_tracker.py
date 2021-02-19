#!/usr/bin/env python3
import numpy as np
import rospy

from roscopter_msgs.msg import TrajectoryCommand
from rosflight_msgs.msg import Command
from nav_msgs.msg import Odometry

class TrajectoryTracker():

    def __init__(self):
        self.odometry_subscriber = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.trajectory_command_subscriber = rospy.Subscriber('trajectory_command', TrajectoryCommand, self.commandCallback, queue_size=5)
        self.rosflight_command_publisher = rospy.Publisher('rosflight_command', Command, queue_size=5, latch = True)
        self.position_gain = np.eye(3)*2
        self.velocity_ain = np.eye(3)*.5
        self.rotation_gain = np.eye(3)*1
        self.gravity = 9.8
        self.mass = 
        self.position = 
        self.velocity = 
        self.acceleration = 
        self.attitude_in_SO3 = 
        while not rospy.is_shutdown():
            rospy.spin()

    def calculateDesired

    def calculateDesiredAttitudeSO3(self, desired_position,desired_velocity,desired_acceleration,desired_heading):
        desired_force = calculateDesiredForce(desired_position,desired_velocity,desired_acceleration)
        desired_body_z_axis = calculateDesiredBodyZAxis(desired_force)
        desired_body_y_axis = calculateDesiredBodyYAxis(desired_body_z_axis,desired_heading)
        desired_body_x_axis = calculateDesiredBodyXAxis(desired_body_y_axis,desired_body_z_axis)
        desired_rotation = np.concatenate( (np.concatenate((desired_body_x_axis,desired_body_y_axis),1) , desired_body_z_axis) , 1)
        return desired_rotation

    def calculateDesiredForce(self, desired_position,desired_velocity,desired_acceleration):
        position_error = calculatePositionError(desired_position)
        velocity_error = calculateVelocityError(desired_velocity)
        gravityVector = np.array([[0],[0],[1]])*self.gravity
        desired_force = -np.dot(self.position_gain,position_error) - np.dot(self.velocity_gain,velocity_error) \
                        - self.mass*gravityVector + self.mass*desired_acceleration
        return desired_force

    def calculateDesiredBodyZAxis(self, desired_force):
        desired_body_z_axis = desired_force / np.linalg.norm(desired_force)
        return desired_body_z_axis

    def calculateDesiredBodyYAxis(self,desired_body_z_axis,desired_heading):
        desired_heading_vector = calculateDesiredHeadingVector(desired_heading)
        desired_body_y_vector =  np.cross(desired_body_z_axis.flatten() , desired_heading_vector().flatten())[:,None]
        desired_body_y_axis = desired_body_y_vector / np.linalg.norm(desired_body_y_vector)
        return desired_body_y_axis

    def calculateDesiredBodyXAxis(self,desired_body_y_axis,desired_body_z_axis):
        desired_body_x_axis = np.cross(desired_body_y_axis.flatten(),desired_body_z_axis.flatten())[:,None]
        return desired_body_x_axis

    def calculateDesiredHeadingVector(self,desired_heading):
        desired_heading_vector = np.array([[np.cos(desired_heading)],[np.sin(desired_heading)],[0]])
        return desired_heading_vector

    def calculatePositionError(self, desired_position):
        position_error = self.position - desired_position
        return position_error

    def calculateVelocityError(self, desired_velocity):
        velocity_error = self.velocity - desired_velocity
        return velocity_error

    def calculateDesiredThrust(self,desired_force):
        body_z_axis = np.array([[0],[0],[1]])
        body_z_axis_in_inertial_frame = np.dot(self.attitudeInSO3,body_z_axis)
        desired_thrust = -np.dot(desired_force , body_z_axis_in_inertial_frame.flatten())
        return desired_thrust

    def calculateThrottleCommand(self, desired_thrust):


    def calculateBodyAngularRateCommands(self, desired_attitude_SO3, derivative_desired_attitude_SO3):
        inertial_to_body_frame_rotation = np.transpose(self.attitude_in_SO3)
        desired_to_inertial_frame_rotation = desired_attitude_SO3
        desired_to_body_frame_rotation = np.dot(inertial_to_body_frame_rotation,desired_to_inertial_frame_rotation)
        attitude_error = calculateAttitudeError(desiredAttitudeSO3)
        desired_body_angular_rates = calculateDesiredBodyAngularRates(desired_attitude_SO3, derivative_desired_attitude_SO3)
        feedforward = np.dot(desired_to_body_frame_rotation,desired_body_angular_rates)
        feedback = np.dot(self.rotation_gain , attitude_error)
        body_angular_rate_commands = feedforward - feedback
        return body_angular_rate_commands

    def calculateAttitudeError(self, desiredAttitudeSO3):
        desired_to_inertial_frame_rotation = desiredAttitudeSO3
        inertial_to_desired_frame_rotation = np.transpose(desired_to_inertial_frame_rotation)
        body_to_inertial_frame_rotation = self.attitude_in_SO3
        inertial_to_body_frame_rotation = np.transpose(body_to_inertial_frame_rotation)
        attitude_error_SO3 = np.dot(inertial_to_desired_frame_rotation,body_to_inertial_frame_rotation) \
                             - np.dot(inertial_to_body_frame_rotation , desired_to_inertial_frame_rotation)
        attitude_error = veeOperator(attitude_error_SO3)/2
        return attitude_error

    def veeOperator(self , skewSymmetricMatrix):
        return np.array([[skewSymmetricMatrix[2,1]] , [skewSymmetricMatrix[0,2]] , [skewSymmetricMatrix[1,0]]]])
        
    def calculateDesiredBodyAngularRates(self, desired_attitude_SO3, derivative_desired_attitude_SO3):
        inertial_to_desired_frame_rotation = np.transpose(desired_attitude_SO3)
        desired_body_angular_rates = veeOperator(inertial_to_desired_frame_rotation,derivative_desired_attitude_SO3)
        return desired_body_angular_rates

    def calculateDerivativeDesiredAttitudeSO3(self, desired_attitude_SO3, desired_position, desired_velocity, desired_acceleration, desired_jerk, desired_heading, desired_heading_rate):
        desired_force = calculateDesiredForce(desired_position,desired_velocity,desired_acceleration)
        derivative_desired_force = calculateDerivativeDesiredForce(desired_velocity, desired_acceleration, desired_jerk)
        desired_body_z_axis = desired_attitude_SO3[:,2][:,None]
        desired_body_y_axis = desired_attitude_SO3[:,1][:,None]
        derivative_desired_body_z_axis = calculateDerivativeDesiredBodyZAxis(desired_force, derivative_desired_force)
        derivative_desired_body_y_axis = calculateDerivativeDesiredBodyYAxis(desired_body_z_axis, derivative_desired_body_z_axis, desired_heading, desired_heading_rate)
        derivative_desired_body_x_axis = calculateDerivativeDesiredBodyXAxis(desired_body_z_axis, derivative_desired_body_z_axis, desired_body_y_axis, derivative_desired_body_y_axis)
        columns_one_and_two = np.concatenate((derivative_desired_body_x_axis,derivative_desired_body_y),1)
        derivative_desired_attitude_SO3 = np.concatenate((columns_one_and_two,derivative_desired_body_z_axis),1)
        return derivative_desired_attitude_SO3

    def calculateDerivativeDesiredBodyZAxis(self, desired_force, derivative_desired_force):
        norm_desired_force = np.linalg.norm(desired_force)
        term_1 = -derivative_desired_force / norm_desired_force
        term_2 = np.dot(desired_force.flatten(),derivative_desired_force)
        term_3 = desired_force * term_2 / (norm_desired_force**3)
        derivative_desired_body_z_axis = term_1 + term_2
        return derivative_desired_body_z_axis

    def calculateDerivativeDesiredBodyYAxis(self, desired_body_z_axis, derivative_desired_body_z_axis, desired_heading, desired_heading_rate):
        desired_heading_vector = calculateDesiredHeadingVector(desired_heading)
        derivative_desired_heading_vector = calculateDerivativeDesiredHeadingVector(desired_heading,desired_heading_rate)
        term_1 = np.cross(derivative_desired_body_z_axis.flatten() , desired_heading_vector.flatten())
        term_2 = np.cross(desired_body_z_axis.flatten(), derivative_desired_heading_vector.flatten())
        derivative_desired_body_y_axis = term_1 + term_2
        return derivative_desired_body_y_axis

    def calculateDerivativeDesiredBodyXAxis(self, desired_body_z_axis, derivative_desired_body_z_axis, desired_body_y_axis, derivative_desired_body_y_axis):
        term_1 = np.cross(derivative_desired_body_y_axis.flatten() , desired_body_z_axis.flatten())
        term_2 = np.cross(desired_body_y_axis.flatten(), derivative_desired_body_z_axis.flatten())
        return term_1 + term_2

    def calculateDerivativeDesiredHeadingVector(self,desired_heading,desired_heading_rate):
        return np.array([[-np.sin(desired_heading)*desired_heading_rate],[np.cos(desired_heading)*desired_heading_rate],[0]])

    def odometryCallback(self, msg):
        north_position = msg.pose.pose.position.x
        east_position = msg.pose.pose.position.y
        down_position = msg.pose.pose.position.z
        current_position = np.array([self.n, self.e, self.d])
        # orientation in quaternion form
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        # yaw from quaternion
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

    def commandCallback(self,msg):
        positions_command = np.array([msg.x_position, msg.y_position, msg.z_position])
        velocities_command = np.array([msg.x_velocity, msg.y_velocity, msg.z_velocity])
        accelerations_command = np.array([msg.x_acceleration, msg.y_acceleration, msg.z_acceleration])
        jerk_command = np.array([msg.x_jerk, msg.y_jerk, msg.z_jerk])
        heading_command = msg.heading
        heading_rate_command = msg.heading_rate

    def publishRosflightCommand(self):
        cmd_msg = Command()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.mode = 1
        cmd_msg.ignore = 0
        cmd_msg.x = 0
        cmd_msg.y = 0
        cmd_msg.z = 2
        cmd_msg.F = .5
        self.rosflight_command_publisher.publish(self.cmd_msg)


if __name__ == '__main__':
    rospy.init_node('trajectory_tracker', anonymous=True)
    try:
        traj_tracker = TrajectoryTracker()
    except:
        rospy.ROSInterruptException
    pass