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
        self.equilibrium_throttle = 0.5
        self.gravity = 9.8
        self.mass = 
        self.position = 
        self.velocity = 
        self.acceleration = 
        self.attitude_in_SO3 = 
        self.desired_position = 
        self.desired_velocity = 
        self.desired_acceleration = 
        self.desired_jerk = 
        self.desired_heading = 
        self.desired_heading_rate =
        while not rospy.is_shutdown():
            rospy.spin()

    def computeLowLevelCommands():
        desired_attitude_SO3 = computeDesiredAttitudeSO3(desired_position,desired_velocity,desired_acceleration,desired_heading)
        derivative_desired_attitude_SO3 = computeDerivativeDesiredAttitudeSO3(desired_attitude_SO3, desired_position, desired_velocity, desired_acceleration, desired_jerk, desired_heading, desired_heading_rate):
        body_angular_rate_commands = computeBodyAngularRateCommands(desired_attitude_SO3, derivative_desired_attitude_SO3)
        desired_force_vector = computeDesiredForceVector(desired_position,desired_velocity,desired_acceleration)
        desired_thrust = computeDesiredThrust(desired_force_vector)
        throtte_command = computeThrottleCommand(desired_thrust)



    def computeDesiredAttitudeSO3(self, desired_position,desired_velocity,desired_acceleration,desired_heading):
        desired_force = computeDesiredForceVector(desired_position,desired_velocity,desired_acceleration)
        desired_body_z_axis = computeDesiredBodyZAxis(desired_force)
        desired_body_y_axis = computeDesiredBodyYAxis(desired_body_z_axis,desired_heading)
        desired_body_x_axis = computeDesiredBodyXAxis(desired_body_y_axis,desired_body_z_axis)
        desired_rotation = np.concatenate( (np.concatenate((desired_body_x_axis,desired_body_y_axis),1) , desired_body_z_axis) , 1)
        return desired_rotation

    def computeDesiredForceVector(self, desired_position,desired_velocity,desired_acceleration):
        position_error = computePositionError(desired_position)
        velocity_error = computeVelocityError(desired_velocity)
        gravityVector = np.array([[0],[0],[1]])*self.gravity
        desired_force = -np.dot(self.position_gain,position_error) - np.dot(self.velocity_gain,velocity_error) \
                        - self.mass*gravityVector + self.mass*desired_acceleration
        return desired_force

    def computeDesiredBodyZAxis(self, desired_force):
        desired_body_z_axis = desired_force / np.linalg.norm(desired_force)
        return desired_body_z_axis

    def computeDesiredBodyYAxis(self,desired_body_z_axis,desired_heading):
        desired_heading_vector = computeDesiredHeadingVector(desired_heading)
        desired_body_y_vector =  np.cross(desired_body_z_axis.flatten() , desired_heading_vector().flatten())[:,None]
        desired_body_y_axis = desired_body_y_vector / np.linalg.norm(desired_body_y_vector)
        return desired_body_y_axis

    def computeDesiredBodyXAxis(self,desired_body_y_axis,desired_body_z_axis):
        desired_body_x_axis = np.cross(desired_body_y_axis.flatten(),desired_body_z_axis.flatten())[:,None]
        return desired_body_x_axis

    def computeDesiredHeadingVector(self,desired_heading):
        desired_heading_vector = np.array([[np.cos(desired_heading)],[np.sin(desired_heading)],[0]])
        return desired_heading_vector

    def computePositionError(self, desired_position):
        position_error = self.position - desired_position
        return position_error

    def computeVelocityError(self, desired_velocity):
        velocity_error = self.velocity - desired_velocity
        return velocity_error

    def computeDesiredThrust(self,desired_force_vector):
        body_z_axis = np.array([[0],[0],[1]])
        body_z_axis_in_inertial_frame = np.dot(self.attitudeInSO3,body_z_axis)
        desired_thrust = -np.dot(desired_force , body_z_axis_in_inertial_frame.flatten())
        return desired_thrust

    def computeThrottleCommand(self, desired_thrust):
        #assumes a linear throttle -> thrust model
        desired_z_acceleration = desired_thrust/self.mass
        max_z_acceleration = self.gravity / self.equilibrium_throttle
        throttle_command = desired_z_acceleration/max_z_acceleration
        saturated_throttle_command = np.clip(throttle_command,0,1)
        return saturated_throttle_command

    def computeBodyAngularRateCommands(self, desired_attitude_SO3, derivative_desired_attitude_SO3):
        inertial_to_body_frame_rotation = np.transpose(self.attitude_in_SO3)
        desired_to_inertial_frame_rotation = desired_attitude_SO3
        desired_to_body_frame_rotation = np.dot(inertial_to_body_frame_rotation,desired_to_inertial_frame_rotation)
        attitude_error = computeAttitudeError(desiredAttitudeSO3)
        desired_body_angular_rates = computeDesiredBodyAngularRates(desired_attitude_SO3, derivative_desired_attitude_SO3)
        feedforward = np.dot(desired_to_body_frame_rotation,desired_body_angular_rates)
        feedback = np.dot(self.rotation_gain , attitude_error)
        body_angular_rate_commands = feedforward - feedback
        return body_angular_rate_commands

    def computeAttitudeError(self, desiredAttitudeSO3):
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
        
    def computeDesiredBodyAngularRates(self, desired_attitude_SO3, derivative_desired_attitude_SO3):
        inertial_to_desired_frame_rotation = np.transpose(desired_attitude_SO3)
        desired_body_angular_rates = veeOperator(inertial_to_desired_frame_rotation,derivative_desired_attitude_SO3)
        return desired_body_angular_rates

    def computeDerivativeDesiredAttitudeSO3(self, desired_attitude_SO3, desired_position, desired_velocity, desired_acceleration, desired_jerk, desired_heading, desired_heading_rate):
        desired_force = computeDesiredForceVector(desired_position,desired_velocity,desired_acceleration)
        derivative_desired_force = computeDerivativeDesiredForce(desired_velocity, desired_acceleration, desired_jerk)
        desired_body_z_axis = desired_attitude_SO3[:,2][:,None]
        desired_body_y_axis = desired_attitude_SO3[:,1][:,None]
        derivative_desired_body_z_axis = computeDerivativeDesiredBodyZAxis(desired_force, derivative_desired_force)
        derivative_desired_body_y_axis = computeDerivativeDesiredBodyYAxis(desired_body_z_axis, derivative_desired_body_z_axis, desired_heading, desired_heading_rate)
        derivative_desired_body_x_axis = computeDerivativeDesiredBodyXAxis(desired_body_z_axis, derivative_desired_body_z_axis, desired_body_y_axis, derivative_desired_body_y_axis)
        columns_one_and_two = np.concatenate((derivative_desired_body_x_axis,derivative_desired_body_y),1)
        derivative_desired_attitude_SO3 = np.concatenate((columns_one_and_two,derivative_desired_body_z_axis),1)
        return derivative_desired_attitude_SO3

    def computeDerivativeDesiredForce(self, desired_velocity, desired_acceleration, desired_jerk):
        velocity_error = computeVelocityError(desired_velocity)
        acceleration_error = computeAccelerationError(desired_acceleration)
        derivative_desired_force = self.mass*desired_jerk - self.position_gain*velocity_error - self.velocity_gain*acceleration_error
        return derivative_desired_force

    def computeDerivativeDesiredBodyZAxis(self, desired_force, derivative_desired_force):
        norm_desired_force = np.linalg.norm(desired_force)
        term_1 = -derivative_desired_force / norm_desired_force
        term_2 = np.dot(desired_force.flatten(),derivative_desired_force)
        term_3 = desired_force * term_2 / (norm_desired_force**3)
        derivative_desired_body_z_axis = term_1 + term_2
        return derivative_desired_body_z_axis

    def computeDerivativeDesiredBodyYAxis(self, desired_body_z_axis, derivative_desired_body_z_axis, desired_heading, desired_heading_rate):
        desired_heading_vector = computeDesiredHeadingVector(desired_heading)
        derivative_desired_heading_vector = computeDerivativeDesiredHeadingVector(desired_heading,desired_heading_rate)
        term_1 = np.cross(derivative_desired_body_z_axis.flatten() , desired_heading_vector.flatten())
        term_2 = np.cross(desired_body_z_axis.flatten(), derivative_desired_heading_vector.flatten())
        derivative_desired_body_y_axis = term_1 + term_2
        return derivative_desired_body_y_axis

    def computeDerivativeDesiredBodyXAxis(self, desired_body_z_axis, derivative_desired_body_z_axis, desired_body_y_axis, derivative_desired_body_y_axis):
        term_1 = np.cross(derivative_desired_body_y_axis.flatten() , desired_body_z_axis.flatten())
        term_2 = np.cross(desired_body_y_axis.flatten(), derivative_desired_body_z_axis.flatten())
        return term_1 + term_2

    def computeDerivativeDesiredHeadingVector(self,desired_heading,desired_heading_rate):
        return np.array([[-np.sin(desired_heading)*desired_heading_rate],[np.cos(desired_heading)*desired_heading_rate],[0]])

    def finiteDifferencing(self, value,new_value,time_step):
        return (new_value - value)/time_step

    def quaternionToSO3(self,q0,q1,q2,q3):
        r00 = 2 * (q0*q0 + q1*q1) - 1
        r01 = 2 * (q1*q2 - q0*q3)
        r02 = 2 * (q1*q3 + q0*q2)
        r10 = 2 * (q1*q2 + q0*q3)
        r11 = 2 * (q0*q0 + q2*q2) - 1
        r12 = 2 * (q2*q3 - q0*q1)
        r20 = 2 * (q1*q3 - q0*q2)
        r21 = 2 * (q2*q3 + q0*q1)
        r22 = 2 * (q0*q0 + q3*q3) - 1
        matrix_SO3 = np.array([[r00, r01, r02],
                                        [r10, r11, r12],
                                        [r20, r21, r22]])
        return matrix_SO3

    def odometryCallback(self, msg):

        # if(self.start_time == 0):
        #     self.clock_start_time = msg.header.stamp
       
        # previous_time = self.current_time
        # self.current_time = msg.header.stamp - self.clock_start_time
        # self.time_step = self.current_time - previous_time
        # if self.time_step <= 0:
        #     return

        north_position = msg.pose.pose.position.x
        east_position = msg.pose.pose.position.y
        down_position = msg.pose.pose.position.z

        north_velocity = msg.twist.twist.linear.x
        east_velocity = msg.twist.twist.linear.y
        down_velocity = msg.twist.twist.linear.z

        north_acceleration = finiteDifferencing(north_velocity, self.velocity.item(0), self.time_step)
        east_acceleration = finiteDifferencing(east_velocity, self.velocity.item(1), self.time_step)
        down_acceleration = finiteDifferencing(down_velocity, self.velocity.item(2), self.time_step)

        self.position = np.array([[north_position], [east_position], [down_position]])
        self.velocity = np.array([[north_velocity], [east_velocty], [down_velocty]])
        self.acceleration = np.array([north_acceleration, east_acceleration, down_acceleration])

        # orientation in quaternion form
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        self.attitude_in_SO3 = quaternionToSO3(qw,qx,qy,qz)


    def commandCallback(self,msg):
        self.desired_position = np.array([msg.x_position, msg.y_position, msg.z_position])
        velocities_command = np.array([msg.x_velocity, msg.y_velocity, msg.z_velocity])
        accelerations_command = np.array([msg.x_acceleration, msg.y_acceleration, msg.z_acceleration])
        jerk_command = np.array([msg.x_jerk, msg.y_jerk, msg.z_jerk])
        heading_command = msg.heading
        heading_rate_command = msg.heading_rate

    def publishRosflightCommand(self , roll_rate, pitch_rate, yaw_rate, throttle):
        cmd_msg = Command()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
        cmd_msg.ignore = IGNORE_NONE
        cmd_msg.x = roll_rate
        cmd_msg.y = pitch_rate
        cmd_msg.z = yaw_rate
        cmd_msg.F = throttle
        self.rosflight_command_publisher.publish(self.cmd_msg)


if __name__ == '__main__':
    rospy.init_node('trajectory_tracker', anonymous=True)
    try:
        traj_tracker = TrajectoryTracker()
    except:
        rospy.ROSInterruptException
    pass