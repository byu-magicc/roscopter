#!/usr/bin/env python3
import numpy as np
import rospy

class TrajectoryTracker():

    def __init__(self):
        self.position_gain = np.array([[1, 0,0] , [0,1,0], [0,0,1]])*4
        self.velocity_gain = np.eye(3)*3
        self.angle_gain = np.array([[1, 0,0] , [0,1,0], [0,0,1]])*8
        self.equilibrium_throttle = 0.65
        self.gravity = 9.8
        self.mass = 3.69
        self.position = np.array([[0],[0],[0]])
        self.velocity = np.array([[0],[0],[0]])
        self.acceleration = np.array([[0],[0],[0]])
        self.jerk = np.array([[0],[0],[0]])
        self.heading = 0
        self.heading_rate = 0
        self.attitude_in_SO3 = np.eye(3)
        self.desired_position = np.array([[0],[0],[0]])
        self.desired_velocity = np.array([[0],[0],[0]])
        self.desired_acceleration = np.array([[0],[0],[0]])
        self.desired_jerk = np.array([[0],[0],[0]])
        self.desired_heading = 0
        self.desired_heading_rate = 0

    def updateState(self, position, velocity, attitude_as_quaternion, time_step):
        self.position = position

        previous_velocity = self.velocity
        bodToInertialFrame = self.attitude_in_SO3
        self.velocity = np.dot(bodToInertialFrame,velocity)
        # self.velocity = velocity

        previous_acceleration = self.acceleration
        north_acceleration = self.finiteDifferencing(self.velocity.item(0), previous_velocity.item(0), time_step)
        east_acceleration = self.finiteDifferencing(self.velocity.item(1), previous_velocity.item(1), time_step)
        down_acceleration = self.finiteDifferencing(self.velocity.item(2), previous_velocity.item(2), time_step)
        self.acceleration = np.array([[north_acceleration], [east_acceleration], [down_acceleration]])

        north_jerk = self.finiteDifferencing(north_acceleration, previous_acceleration.item(0), time_step)
        east_jerk = self.finiteDifferencing(east_acceleration, previous_acceleration.item(1), time_step)
        down_jerk = self.finiteDifferencing(down_acceleration, previous_acceleration.item(2), time_step)
        self.jerk = np.array([[north_jerk], [east_jerk], [down_jerk]])

        self.attitude_in_SO3 = self.quaternionToSO3(attitude_as_quaternion)

        previous_heading = self.heading
        self.heading = self.getCurrentHeading()
        self.heading_rate = self.finiteDifferencing(self.heading,previous_heading, time_step)

    def updateDesiredState(self, desired_position, desired_velocity, desired_acceleration, desired_jerk, desired_heading, desired_heading_rate):
        self.desired_position = desired_position
        self.desired_velocity = desired_velocity
        self.desired_acceleration = desired_acceleration
        self.desired_jerk = desired_jerk
        self.desired_heading = desired_heading
        self.desired_heading_rate = desired_heading_rate

    def computeRosflightCommand(self):
        #calculate linear errors
        position_error = self.computeLinearError(self.position, self.desired_position)
        velocity_error = self.computeLinearError(self.velocity, self.desired_velocity)
        acceleration_error = self.computeLinearError(self.acceleration, self.desired_acceleration)

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
        throttle = self.computeThrottleCommand(desired_thrust)
        roll_rate = body_angular_rate_commands.item(0)
        pitch_rate = body_angular_rate_commands.item(1)
        yaw_rate = body_angular_rate_commands.item(2)
        rosflight_command = np.array([roll_rate, pitch_rate, yaw_rate , throttle])
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
        body_angular_rate_commands = - feedback + feedforward
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
        desired_body_z_axis = -desired_force / np.linalg.norm(desired_force)
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
        desired_force_vector = -np.dot(self.position_gain,position_error) - np.dot(self.velocity_gain,velocity_error) \
                        - self.mass*gravityVector + self.mass*self.desired_acceleration
        return desired_force_vector

    def computeDerivativeDesiredHeadingVector(self):
        return np.array([[-np.sin(self.desired_heading)*self.desired_heading_rate],[np.cos(self.desired_heading)*self.desired_heading_rate],[0]])

    def computeDesiredHeadingVector(self):
        desired_heading_vector = np.array([[np.cos(self.desired_heading)],[np.sin(self.desired_heading)],[0]])
        return desired_heading_vector

    def computeLinearError(self, current_state, desired_state):
        error = current_state - desired_state
        return error

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