from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt 
import numpy as np
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Circle, Arc
from scipy.spatial.transform import Rotation as R
from lqr_controller import LQRController

def arcData(arc_mag , ortho_axis):
    if ortho_axis == "x":
        xdata = np.zeros(100)
        ydata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        zdata = np.sin(np.linspace(0,np.pi*2,100)*arc_mag)

    elif ortho_axis == "y":
        xdata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        ydata = np.zeros(100)
        zdata = -np.sin(np.linspace(0,np.pi*2,100)*arc_mag)

    else:
        xdata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        ydata = np.sin(np.linspace(0,np.pi*2,100)*arc_mag)
        zdata = np.zeros(100)
    return xdata, ydata, zdata

#initialize trajectory tracker
lqr_control = LQRController()

#initialize states
x_pos = 0
y_pos = 0
z_pos = 0
x_vel = 0
y_vel = 0
z_vel = 0
attitude_as_quat = np.array([1,0,0,0])
time_step = 0.01
position = np.array([[x_pos],[y_pos],[z_pos]])
velocity = np.array([[x_vel],[y_vel],[z_vel]])
lqr_control.updateState(position,velocity, attitude_as_quat, time_step)
x_accel = lqr_control.acceleration.item(0)
y_accel = lqr_control.acceleration.item(1)
z_accel = lqr_control.acceleration.item(2)
xdir = np.dot(np.array([1,0,0]),lqr_control.attitude_in_SO3)
ydir = np.dot(np.array([0,1,0]),lqr_control.attitude_in_SO3)
zdir = np.dot(np.array([0,0,1]),lqr_control.attitude_in_SO3)

#initialize desired states
t = 3
radius = 2
height = 4
z_pos_des = -height
x_pos_des = 2 #np.sin(t)*radius
y_pos_des = 2 #np.cos(t)*radius
z_vel_des = 0
x_vel_des = 1#np.cos(t)*radius
y_vel_des = 1#-np.sin(t)*radius
z_accel_des = .1
x_accel_des = 0#-np.sin(t)*radius
y_accel_des = .5#-np.cos(t)*radius
z_jerk_des = .0
x_jerk_des = 0.0 #-np.cos(t)*radius
y_jerk_des = 0.1 #np.sin(t)*radius
desired_heading = 0
desired_heading_rate = 0
desired_position = np.array([[x_pos_des], [y_pos_des], [z_pos_des]])
desired_velocity = np.array([[x_vel_des], [y_vel_des], [z_vel_des]])
desired_acceleration = np.array([[x_accel_des], [y_accel_des], [z_accel_des]])
desired_jerk = np.array([[x_jerk_des], [y_jerk_des], [z_jerk_des]])
lqr_control.updateDesiredState(desired_position, desired_velocity, desired_acceleration, desired_jerk, desired_heading, desired_heading_rate)

# # calculate commands and desired values
desired_accel_vec = lqr_control.computeDesiredThrustAccelerationVector() 
desired_accel_vec = lqr_control.computeDesiredThrustAccelerationVector() / np.linalg.norm(desired_accel_vec)
desired_heading_vec = lqr_control.computeDesiredHeadingVector()
derivative_desired_heading_vec = lqr_control.computeDerivativeDesiredHeadingVector()
desired_rotation = lqr_control.computeDesiredAttitudeSO3(desired_accel_vec)
desired_xdir = np.dot(desired_rotation, np.array([1,0,0]))
desired_ydir = np.dot(desired_rotation, np.array([0,1,0]))
desired_zdir = np.dot(desired_rotation, np.array([0,0,1]))
derivative_desired_rotation = lqr_control.computeDerivativeDesiredAttitude(desired_accel_vec, derivative_desired_heading_vec)
desired_body_angular_rates = lqr_control.computeDesiredBodyAngularRates(derivative_desired_rotation)
des_pitch_rate = desired_body_angular_rates.item(1)
des_yaw_rate = desired_body_angular_rates.item(2)
des_roll_rate = desired_body_angular_rates.item(0)
rosflight_command = lqr_control.computeRosflightCommand()
roll_rate_command = rosflight_command.item(0)
pitch_rate_command = rosflight_command.item(1)
yaw_rate_command = rosflight_command.item(2)
throttle_command = rosflight_command.item(3)

fig = plt.figure()
ax = plt.axes(projection='3d')
min_lim = -5
max_lim = 5
ax.set_xlim3d(min_lim, max_lim)
ax.set_ylim3d(min_lim, max_lim)
ax.set_zlim3d(min_lim, max_lim)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.legend()

##### position ######
ax.scatter3D(x_pos, y_pos, z_pos,color='black',label='pos')
ax.scatter3D(x_pos_des, y_pos_des, z_pos_des, color='green',label='des_pos')

##### orientation ######
#desired heading
ax.plot3D([x_pos_des, x_pos_des + desired_heading_vec.item(0)], [y_pos_des, y_pos_des + desired_heading_vec.item(1)], [z_pos_des, z_pos_des + desired_heading_vec.item(2)], color='b', linestyle='-',label='des heading')
ax.plot3D([x_pos_des + desired_heading_vec.item(0) , x_pos_des + desired_heading_vec.item(0) - desired_heading_vec.item(1)*np.sign(desired_heading_rate)],\
          [y_pos_des + desired_heading_vec.item(1) , y_pos_des + desired_heading_vec.item(1) + desired_heading_vec.item(0)*np.sign(desired_heading_rate) ],\
          [z_pos_des + desired_heading_vec.item(2) , z_pos_des + desired_heading_vec.item(2) + desired_heading_vec.item(2) ], color='b', linestyle='-',label='des heading_rate')
#desired acceleration due to thrust
ax.plot3D([x_pos_des, x_pos_des + desired_accel_vec.item(0)], [y_pos_des, y_pos_des + desired_accel_vec.item(1)], [z_pos_des, z_pos_des + desired_accel_vec.item(2)], color='b', linestyle='-',label='des acceleration')
# desired jerk
ax.plot3D([x_pos_des + desired_accel_vec.item(0) , x_pos_des + desired_accel_vec.item(0) + desired_jerk.item(0)]\
         ,[y_pos_des + desired_accel_vec.item(1) , y_pos_des + desired_accel_vec.item(1) + desired_jerk.item(1)]\
         ,[z_pos_des + desired_accel_vec.item(2) , z_pos_des + desired_accel_vec.item(2) + desired_jerk.item(2)], color='b', linestyle='-',label='des jerk dir')
#x dir
ax.plot3D([x_pos, x_pos + xdir.item(0)], [y_pos, y_pos + xdir.item(1)], [z_pos, z_pos + xdir.item(2)], color='black', linestyle='-',label='x_dir')
#desired x dir
ax.plot3D([x_pos_des, x_pos_des + desired_xdir.item(0)], [y_pos_des, y_pos_des + desired_xdir.item(1)], [z_pos_des, z_pos_des + desired_xdir.item(2)], color='green', linestyle='-',label='des x_dir')
#z dir
ax.plot3D([x_pos, x_pos + zdir.item(0)], [y_pos, y_pos + zdir.item(1)], [z_pos, z_pos + zdir.item(2)], color='black', linestyle='-',label='z_dir')
#desired z dir
ax.plot3D([x_pos_des, x_pos_des + desired_zdir.item(0)], [y_pos_des, y_pos_des + desired_zdir.item(1)], [z_pos_des, z_pos_des + desired_zdir.item(2)], color='green', linestyle='-',label='des z_dir')

# #y orientation
# ax.plot3D([x_pos, x_pos + ydir.item(0)], [y_pos, y_pos + ydir.item(1)], [z_pos, z_pos + ydir.item(2)], color='gray', linestyle='-',label='y_dir')
# ax.plot3D([x_pos_des, x_pos_des + desired_ydir.item(0)], [y_pos_des, y_pos_des + desired_ydir.item(1)], [z_pos_des, z_pos_des + desired_ydir.item(2)], color='limegreen', linestyle='-',label='des y_dir')

# #velocity
# ax.plot3D([x_pos, x_pos + x_vel], [y_pos, y_pos + y_vel], [z_pos, z_pos + z_vel], color='gray', linestyle='--',label='vel')
# ax.plot3D([x_pos_des, x_pos_des + x_vel_des], [y_pos_des, y_pos_des + y_vel_des], [z_pos_des, z_pos_des + z_vel_des], color='green', linestyle='--',label='des_vel')

# #acceleration
# ax.plot3D([x_pos, x_pos + x_accel], [y_pos, y_pos + y_accel], [z_pos, z_pos + z_accel], color='gray', linestyle=':',label='accel')
# ax.plot3D([x_pos_des, x_pos_des + x_accel_des], [y_pos_des, y_pos_des + y_accel_des], [z_pos_des, z_pos_des + z_accel_des], color='green', linestyle=':',label='des_accel')

# #desired_force
# ax.plot3D([x_pos, x_pos + desired_force_vec.item(0)], [y_pos, y_pos +  desired_force_vec.item(1)], [z_pos, z_pos +  desired_force_vec.item(2)], color='green', linestyle='-.',label='desired_force')

# #thrust_vec
# ax.plot3D([x_pos, x_pos + thrust_vec.item(0)], [y_pos, y_pos +  thrust_vec.item(1)], [z_pos, z_pos +  thrust_vec.item(2)], color='blue', linestyle='-.',label='thrust')

# #rate commands
angle_rate_command = np.sqrt(roll_rate_command**2 + pitch_rate_command**2 + yaw_rate_command**2)
roll_mag_command = roll_rate_command/angle_rate_command
pitch_mag_command = pitch_rate_command/angle_rate_command
yaw_mag_command = yaw_rate_command/angle_rate_command

#roll command
roll_x, roll_y, roll_z = arcData(roll_mag_command , "x")
ax.plot3D(roll_x,roll_y,roll_z,color='red',label='roll_rate_c')
ax.scatter3D(roll_x[-1],roll_y[-1],roll_z[-1],color='red',marker='>')

#pitch command
pitch_x, pitch_y, pitch_z = arcData(pitch_mag_command , "y")
ax.plot3D(pitch_x,pitch_y,pitch_z,color='purple',label='pitch_rate_c')
ax.scatter3D(pitch_x[-1],pitch_y[-1],pitch_z[-1],color='purple',marker='^')

# yaw command
yaw_x, yaw_y, yaw_z = arcData(yaw_mag_command , "z")
ax.plot3D(yaw_x,yaw_y,yaw_z,color='gold',label="yaw_rate_C")
ax.scatter3D(yaw_x[-1],yaw_y[-1],yaw_z[-1],color='gold',marker='<')

#desired body rates
angle_rate_des = np.sqrt(des_roll_rate**2 + des_pitch_rate**2 + des_yaw_rate**2)
des_roll_mag = des_roll_rate / angle_rate_des
des_pitch_mag = des_pitch_rate / angle_rate_des
des_yaw_mag = des_yaw_rate / angle_rate_des

# # desired roll_rate
roll_x_des, roll_y_des, roll_z_des = arcData(des_roll_mag , "x")
roll_data_matrix = np.vstack((roll_x_des, roll_y_des))
roll_data_matrix = np.vstack((roll_data_matrix, roll_z_des))
roll_data_matrix = np.dot(desired_rotation,roll_data_matrix) + np.array([[x_pos_des],[y_pos_des],[z_pos_des]])
roll_x_des = roll_data_matrix[0,:]
roll_y_des = roll_data_matrix[1,:]
roll_z_des = roll_data_matrix[2,:]
ax.plot3D(roll_x_des,roll_y_des,roll_z_des,color='red',label='des_roll_rate_c')
ax.scatter3D(roll_x_des[-1],roll_y_des[-1],roll_z_des[-1],color='red',marker='>')

# #desired pitch_rate
pitch_x_des, pitch_y_des, pitch_z_des = arcData(des_pitch_mag , "y")
pitch_data_matrix = np.vstack((pitch_x_des, pitch_y_des))
pitch_data_matrix = np.vstack((pitch_data_matrix, pitch_z_des))
pitch_data_matrix = np.dot(desired_rotation,pitch_data_matrix) + np.array([[x_pos_des],[y_pos_des],[z_pos_des]])
pitch_x_des = pitch_data_matrix[0,:]
pitch_y_des = pitch_data_matrix[1,:]
pitch_z_des = pitch_data_matrix[2,:]
ax.plot3D(pitch_x_des,pitch_y_des,pitch_z_des,color='purple',label='des_pitch_rate_c')
ax.scatter3D(pitch_x_des[-1],pitch_y_des[-1],pitch_z_des[-1],color='purple',marker='^')

#desired yaw_rate
yaw_x_des, yaw_y_des, yaw_z_des = arcData(des_yaw_mag , "z")
yaw_data_matrix = np.vstack((yaw_x_des, yaw_y_des))
yaw_data_matrix = np.vstack((yaw_data_matrix, yaw_z_des))
yaw_data_matrix = np.dot(desired_rotation,yaw_data_matrix) + np.array([[x_pos_des],[y_pos_des],[z_pos_des]])
yaw_x_des = yaw_data_matrix[0,:]
yaw_y_des = yaw_data_matrix[1,:]
yaw_z_des = yaw_data_matrix[2,:]
ax.plot3D(yaw_x_des,yaw_y_des,yaw_z_des,color='gold',label="des_yaw_rate_C")
ax.scatter3D(yaw_x_des[-1],yaw_y_des[-1],yaw_z_des[-1],color='gold',marker='<')

ax.legend()
plt.show()

