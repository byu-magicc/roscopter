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
        self.altitude = 3
        self.speed = 2

    def get_time(self):
        time = rospy.get_time() - self.start_time
        return time

    def publish_trajectory(self):
        t = self.get_time()
        traj_command = TrajectoryCommand()

        # traj_command.x_position = self.x(t)
        # traj_command.y_position = self.y(t)
        # traj_command.z_position = self.z(t)
        # traj_command.x_velocity = self.dxdt(t)
        # traj_command.y_velocity = self.dydt(t)
        # traj_command.z_velocity = self.dzdt(t)
        # traj_command.x_acceleration = self.dx2dt(t)
        # traj_command.y_acceleration = self.dy2dt(t)
        # traj_command.z_acceleration = self.dz2dt(t)
        # traj_command.x_jerk = self.dx3dt(t)
        # traj_command.y_jerk = self.dy3dt(t)
        # traj_command.z_jerk = self.dz3dt(t)
        # traj_command.heading = self.psi(t)
        # traj_command.heading_rate = self.dpsidt(t)

        traj_command.x_position = 0
        traj_command.y_position = 0
        traj_command.z_position = -4
        traj_command.x_velocity = 0
        traj_command.y_velocity = 0
        traj_command.z_velocity = 0
        traj_command.x_acceleration = 0
        traj_command.y_acceleration = 0
        traj_command.z_acceleration = 0
        traj_command.x_jerk = 0
        traj_command.y_jerk = 0
        traj_command.z_jerk = 0
        traj_command.heading = 0
        traj_command.heading_rate = 0


        self.trajectory_publisher.publish(traj_command)

        
    def x(self, t):
        f = self.radius*np.sin(t/self.speed)
        return f
    
    def dxdt(self, t):
        f = (self.radius/self.speed)*np.cos(t/self.speed)
        return f

    def dx2dt(self, t):
        f = (-self.radius/self.speed**2)*np.sin(t/self.speed)
        return f

    def dx3dt(self, t):
        f = (-self.radius/self.speed**3)*np.cos(t/self.speed)
        return f

    def y(self, t):
        f = self.radius*np.cos(t/self.speed)
        return f

    def dydt(self, t):
        f = (-self.radius/self.speed)*np.sin(t/self.speed)
        return f

    def dy2dt(self, t):
        f = (-self.radius/self.speed**2)*np.cos(t/self.speed)
        return f

    def dy3dt(self, t):
        f = (self.radius/self.speed**3)*np.sin(t/self.speed)
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
        f = np.sin(t/self.speed)
        # f = np.arctan([np.cos(t/self.speed) . np.sin(t/self.speed)])
        return f

    def dpsidt(self, t):
        # f = (1/np.cos(t/self.speed))**2 / (self.speed*np.tan(t/self.speed) + self.speed)
        f = np.sin(t/self.speed)/self.speed
        return f
        
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
