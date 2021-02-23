#!/usr/bin/env python3
import numpy as np
import rospy

# from geometry_msgs import Vector3Stamped
from roscopter_msgs.msg import TrajectoryCommand


class TrajectoryManager():

    def __init__(self):
        self.start_time = rospy.get_time()
        self.current_time = 0
        self.trajectory_publisher = rospy.Publisher("trajectory", TrajectoryCommand, queue_size=5, latch=True)
        self.radius = 5
        self.altitude = 5

    def get_time(self):
        time = rospy.get_time() - self.start_time
        return time

    def publish_trajectory(self):
        t = self.get_time()
        traj_command = TrajectoryCommand()
        traj_command.x_position = self.x(t)
        traj_command.y_position = self.y(t)
        traj_command.z_position = self.z(t)
        traj_command.x_velocity = self.dxdt(t)
        traj_command.y_velocity = self.dydt(t)
        traj_command.z_velocity = self.dzdt(t)
        traj_command.x_acceleration = self.dx2dt(t)
        traj_command.y_acceleration = self.dy2dt(t)
        traj_command.z_acceleration = self.dz2dt(t)
        traj_command.x_jerk = self.dx3dt(t)
        traj_command.y_jerk = self.dy3dt(t)
        traj_command.z_jerk = self.dz3dt(t)
        traj_command.heading = self.psi(t)
        traj_command.heading_rate = self.dpsidt(t)
        self.trajectory_publisher.publish(traj_command)

        
    def x(self, t):
        f = self.radius*np.sin(t/10)
        return f
    
    def dxdt(self, t):
        f = (self.radius/10)*np.cos(t/10)
        return f

    def dx2dt(self, t):
        f = (-self.radius/100)*np.sin(t/10)
        return f

    def dx3dt(self, t):
        f = (-self.radius/1000)*np.cos(t/10)
        return f

    def y(self, t):
        f = self.radius*np.cos(t/10)
        return f

    def dydt(self, t):
        f = (-self.radius/10)*np.sin(t/10)
        return f

    def dy2dt(self, t):
        f = (-self.radius/100)*np.cos(t/10)
        return f

    def dy3dt(self, t):
        f = (self.radius/1000)*np.sin(t/10)
        return f

    def z(self, t):
        return -self.altitude
    
    def dzdt(self, t):
        return 0

    def dz2dt(self, t):
        return 0

    def dz3dt(self, t):
        return 0

    def psi(self, t):
        return 0

    def dpsidt(self, t):
        return 0
        
if __name__ == '__main__':
    rospy.init_node('trajectory_manager', anonymous=True)
    try:
        traj_manager = TrajectoryManager()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            traj_manager.publish_trajectory()
            rate.sleep()
    except:
        rospy.ROSInterruptException
    pass