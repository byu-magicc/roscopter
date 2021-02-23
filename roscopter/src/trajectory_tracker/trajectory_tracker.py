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
        self.velocity_gain = np.eye(3)*.5
        self.angle_gain = np.eye(3)*1
        self.equilibrium_throttle = 0.5
        self.gravity = 9.8
        self.mass = 3.69
        self.position = np.array([[0],[0],[0]])
        self.velocity = np.array([[0],[0],[0]])
        self.acceleration = np.array([[0],[0],[0]])
        self.attitude_in_SO3 = np.eye(3)
        self.desired_position = np.array([[0],[0],[0]])
        self.desired_velocity = np.array([[0],[0],[0]])
        self.desired_acceleration = np.array([[0],[0],[0]])
        self.desired_jerk = np.array([[0],[0],[0]])
        self.desired_heading = 0
        self.desired_heading_rate = 0
        self.start_time = rospy.get_rostime().to_sec()
        self.current_time = 0
        while not rospy.is_shutdown():
            rospy.spin()

    def odometryCallback(self, msg):
        previous_time = self.current_time
        self.current_time = rospy.get_rostime().to_sec() - self.start_time
        time_step = self.current_time - previous_time
        if time_step <= 0:
            return
        north_position = msg.pose.pose.position.x
        east_position = msg.pose.pose.position.y
        down_position = msg.pose.pose.position.z
        self.position = np.array([[north_position], [east_position], [down_position]])

        north_velocity = msg.twist.twist.linear.x
        east_velocity = msg.twist.twist.linear.y
        down_velocity = msg.twist.twist.linear.z
        previous_north_velocity = self.velocity.item(0)
        previous_east_velocity = self.velocity.item(1)
        previous_down_velocity = self.velocity.item(2)
        self.velocity = np.array([[north_velocity], [east_velocity], [down_velocity]])

        north_acceleration = self.finiteDifferencing(north_velocity, previous_north_velocity, time_step)
        east_acceleration = self.finiteDifferencing(east_velocity, previous_east_velocity, time_step)
        down_acceleration = self.finiteDifferencing(down_velocity, previous_down_velocity, time_step)
        self.acceleration = np.array([[north_acceleration], [east_acceleration], [down_acceleration]])

        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        self.attitude_in_SO3 = self.quaternionToSO3(qw,qx,qy,qz)

    def commandCallback(self,msg):
        self.desired_position = np.array([[msg.x_position], [msg.y_position], [msg.z_position]])
        self.desired_velocity = np.array([[msg.x_velocity], [msg.y_velocity], [msg.z_velocity]])
        self.desired_acceleration = np.array([[msg.x_acceleration], [msg.y_acceleration], [msg.z_acceleration]])
        self.desired_jerk = np.array([[msg.x_jerk], [msg.y_jerk], [msg.z_jerk]])
        self.desired_heading = msg.heading
        self.desired_heading_rate = msg.heading_rate
        rosflight_commands = self.computeRosflightCommand()
        self.publishRosflightCommand(rosflight_commands.item(0), rosflight_commands.item(1), rosflight_commands.item(2), rosflight_commands.item(3))

    def publishRosflightCommand(self , roll_rate, pitch_rate, yaw_rate, throttle):
        cmd_msg = Command()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.mode = Command.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
        cmd_msg.ignore = Command.IGNORE_NONE
        cmd_msg.x = roll_rate
        cmd_msg.y = pitch_rate
        cmd_msg.z = yaw_rate
        cmd_msg.F = throttle
        self.rosflight_command_publisher.publish(cmd_msg)

    def computeRosflightCommand(self):
        #calculate linear errors
        position_error = self.computePositionError()
        velocity_error = self.computeVelocityError()
        acceleration_error = self.computeAccelerationError()

        #calculate desired states
        desired_heading_vector = self.computeDesiredHeadingVector()
        derivative_desired_heading_vector = self.computeDerivativeDesiredHeadingVector()
        desired_force_vector = self.computeDesiredForceVector(position_error, velocity_error)
        desired_thrust = self.computeDesiredThrust(desired_force_vector)
        desired_attitude_SO3 = self.computeDesiredAttitudeSO3(desired_force_vector, desired_heading_vector)
        derivative_desired_force_vector = self.computeDerivativeDesiredForceVector(velocity_error, acceleration_error)
        derivative_desired_attitude_SO3 = self.computeDerivativeDesiredAttitudeSO3(desired_force_vector, derivative_desired_force_vector, desired_attitude_SO3, desired_heading_vector, derivative_desired_heading_vector)
        desired_body_angular_rates = self.computeDesiredBodyAngularRates(desired_attitude_SO3, derivative_desired_attitude_SO3)

        #calculate angular errors
        euler_attitude_error = self.computeEulerAttitudeError(desired_attitude_SO3)
        desired_to_body_frame_rotation = self.computeDesiredToBodyFrameRotation(desired_attitude_SO3)

        #compute commanded outputs
        body_angular_rate_commands = self.computeBodyAngularRateCommands(euler_attitude_error, desired_to_body_frame_rotation, desired_body_angular_rates)
        throttle_command = self.computeThrottleCommand(desired_thrust)
        rosflight_command = np.array([body_angular_rate_commands.item(0), body_angular_rate_commands.item(1) , body_angular_rate_commands.item(2) , throttle_command])
        return rosflight_command

    def computeThrottleCommand(self, desired_thrust):
        #assumes a linear throttle -> thrust model
        desired_z_acceleration = desired_thrust/self.mass
        max_z_acceleration = self.gravity / self.equilibrium_throttle
        throttle_command = desired_z_acceleration/max_z_acceleration
        saturated_throttle_command = np.clip(throttle_command,0,1)
        return saturated_throttle_command

    def computeBodyAngularRateCommands(self, euler_attitude_error, desired_to_body_frame_rotation, desired_body_angular_rates):
        feedforward = np.dot(desired_to_body_frame_rotation,desired_body_angular_rates)
        feedback = np.dot(self.angle_gain , euler_attitude_error)
        body_angular_rate_commands = feedforward - feedback
        return body_angular_rate_commands

    def computeEulerAttitudeError(self, desired_attitude_SO3):
        desired_to_inertial_frame_rotation = desired_attitude_SO3
        inertial_to_desired_frame_rotation = np.transpose(desired_to_inertial_frame_rotation)
        body_to_inertial_frame_rotation = self.attitude_in_SO3
        inertial_to_body_frame_rotation = np.transpose(body_to_inertial_frame_rotation)
        attitude_error_SO3 = np.dot(inertial_to_desired_frame_rotation,body_to_inertial_frame_rotation) \
                             - np.dot(inertial_to_body_frame_rotation , desired_to_inertial_frame_rotation)
        attitude_error = self.veeOperator(attitude_error_SO3)/2
        return attitude_error

    def computeDesiredToBodyFrameRotation(self, desired_attitude_SO3):
        inertial_to_body_frame_rotation = np.transpose(self.attitude_in_SO3)
        desired_to_inertial_frame_rotation = desired_attitude_SO3
        desired_to_body_frame_rotation = np.dot(inertial_to_body_frame_rotation,desired_to_inertial_frame_rotation)
        return desired_to_body_frame_rotation

    def computeDesiredBodyAngularRates(self, desired_attitude_SO3, derivative_desired_attitude_SO3):
        inertial_to_desired_frame_rotation = np.transpose(desired_attitude_SO3)
        desired_body_angular_rates = self.veeOperator(np.dot(inertial_to_desired_frame_rotation,derivative_desired_attitude_SO3))
        return desired_body_angular_rates

    def computeDerivativeDesiredForceVector(self, velocity_error, acceleration_error):
        derivative_desired_force = self.mass*self.desired_jerk - np.dot(self.position_gain,velocity_error) - np.dot(self.velocity_gain,acceleration_error)
        return derivative_desired_force

    def computeDerivativeDesiredAttitudeSO3(self, desired_force_vector, derivative_desired_force_vector, desired_attitude_SO3, desired_heading_vector, derivative_desired_heading_vector):
        desired_body_z_axis = desired_attitude_SO3[:,2][:,None]
        desired_body_y_axis = desired_attitude_SO3[:,1][:,None]
        derivative_desired_body_z_axis = self.computeDerivativeDesiredBodyZAxis(desired_force_vector, derivative_desired_force_vector)
        derivative_desired_body_y_axis = self.computeDerivativeDesiredBodyYAxis(desired_body_z_axis, derivative_desired_body_z_axis, desired_heading_vector, derivative_desired_heading_vector)
        derivative_desired_body_x_axis = self.computeDerivativeDesiredBodyXAxis(desired_body_z_axis, derivative_desired_body_z_axis, desired_body_y_axis, derivative_desired_body_y_axis)
        columns_one_and_two = np.concatenate((derivative_desired_body_x_axis,derivative_desired_body_y_axis),1)
        derivative_desired_attitude_SO3 = np.concatenate((columns_one_and_two,derivative_desired_body_z_axis),1)
        return derivative_desired_attitude_SO3

    def computeDerivativeDesiredBodyZAxis(self, desired_force_vector, derivative_desired_force_vector):
        norm_desired_force = np.linalg.norm(desired_force_vector)
        term_1 = -derivative_desired_force_vector / norm_desired_force
        term_2 = np.dot(desired_force_vector.flatten(),derivative_desired_force_vector)
        term_3 = desired_force_vector * term_2 / (norm_desired_force**3)
        derivative_desired_body_z_axis = term_1 + term_3
        return derivative_desired_body_z_axis

    def computeDerivativeDesiredBodyYAxis(self, desired_body_z_axis, derivative_desired_body_z_axis, desired_heading_vector, derivative_desired_heading_vector):
        term_1 = np.cross(derivative_desired_body_z_axis.flatten() , desired_heading_vector.flatten())
        term_2 = np.cross(desired_body_z_axis.flatten(), derivative_desired_heading_vector.flatten())
        derivative_desired_body_y_axis = term_1 + term_2
        return derivative_desired_body_y_axis[:,None]

    def computeDerivativeDesiredBodyXAxis(self, desired_body_z_axis, derivative_desired_body_z_axis, desired_body_y_axis, derivative_desired_body_y_axis):
        term_1 = np.cross(derivative_desired_body_y_axis.flatten() , desired_body_z_axis.flatten())
        term_2 = np.cross(desired_body_y_axis.flatten(), derivative_desired_body_z_axis.flatten())
        derivative_desired_body_x_axis = term_1 + term_2
        return derivative_desired_body_x_axis[:,None]

    def computeDesiredAttitudeSO3(self, desired_force_vector,desired_heading_vector):
        desired_body_z_axis = self.computeDesiredBodyZAxis(desired_force_vector)
        desired_body_y_axis = self.computeDesiredBodyYAxis(desired_body_z_axis,desired_heading_vector)
        desired_body_x_axis = self.computeDesiredBodyXAxis(desired_body_y_axis,desired_body_z_axis)
        desired_rotation = np.concatenate( (np.concatenate((desired_body_x_axis,desired_body_y_axis),1) , desired_body_z_axis) , 1)
        return desired_rotation

    def computeDesiredBodyZAxis(self, desired_force):
        desired_body_z_axis = desired_force / np.linalg.norm(desired_force)
        return desired_body_z_axis

    def computeDesiredBodyYAxis(self,desired_body_z_axis,desired_heading_vector):
        desired_body_y_vector =  np.cross(desired_body_z_axis.flatten() , desired_heading_vector.flatten())[:,None]
        desired_body_y_axis = desired_body_y_vector / np.linalg.norm(desired_body_y_vector)
        return desired_body_y_axis

    def computeDesiredBodyXAxis(self,desired_body_y_axis,desired_body_z_axis):
        desired_body_x_axis = np.cross(desired_body_y_axis.flatten(),desired_body_z_axis.flatten())[:,None]
        return desired_body_x_axis

    def computeDesiredThrust(self,desired_force_vector):
        body_z_axis = np.array([[0],[0],[1]])
        body_z_axis_in_inertial_frame = np.dot(self.attitude_in_SO3, body_z_axis)
        desired_thrust = -np.dot(desired_force_vector.flatten() , body_z_axis_in_inertial_frame.flatten())
        return desired_thrust

    def computeDesiredForceVector(self, position_error, velocity_error):
        gravityVector = np.array([[0],[0],[1]])*self.gravity
        desired_force = -np.dot(self.position_gain,position_error) - np.dot(self.velocity_gain,velocity_error) \
                        - self.mass*gravityVector + self.mass*self.desired_acceleration
        return desired_force

    def computeDerivativeDesiredHeadingVector(self):
        return np.array([[-np.sin(self.desired_heading)*self.desired_heading_rate],[np.cos(self.desired_heading)*self.desired_heading_rate],[0]])

    def computeDesiredHeadingVector(self):
        desired_heading_vector = np.array([[np.cos(self.desired_heading)],[np.sin(self.desired_heading)],[0]])
        return desired_heading_vector

    #make these one function?
    def computePositionError(self):
        position_error = self.position - self.desired_position
        return position_error

    def computeVelocityError(self):
        velocity_error = self.velocity - self.desired_velocity
        return velocity_error

    def computeAccelerationError(self):
        acceleration_error = self.acceleration - self.desired_acceleration
        return acceleration_error

    def veeOperator(self , skewSymmetricMatrix):
        return np.array([[skewSymmetricMatrix[2,1]] , [skewSymmetricMatrix[0,2]] , [skewSymmetricMatrix[1,0]]])

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


if __name__ == '__main__':
    rospy.init_node('trajectory_tracker', anonymous=True)
    try:
        traj_tracker = TrajectoryTracker()
    except:
        rospy.ROSInterruptException
    pass