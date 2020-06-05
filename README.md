ROScopter
=========

This repository contains the ROS packages associated with flying a multirotor on the [ROSflight](http://rosflight.org) autopilot stack. The `roscopter` ROS package contains a high-level multirotor autopilot while the `roscopter_sim` package is used for simulation.

### RUNNING SIMULATION ###

# Simplified Dynamics Model

Clone this repo, the [ROSflight_plugins](https://github.com/byu-magicc/rosflight_plugins) repo, and the [ROSflight](https://github.com/rosflight/rosflight.git) repo into `catkin_ws/src`, you can run the Gazebo simulator with

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter_sim multirotor.launch
```
This simulation does not simulate the firmware and does not require a joystick. It has the same API as the SIL stack, and can be useful for developing high-level algorithms, however it makes some simplifying assumptions in the dynamics which reduces the fidelity of the simulation.

# Software In The Loop
To run the `rosflight` SIL simulation [information](http://docs.rosflight.org/en/latest/user-guide/gazebo_simulation/) with the full roscopter stack, clone the [ROSflight](github.com/rosflight/rosflight) repo, connect a joystick or transmitter and run

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter copter_sil.launch
```


### CORE AUTOPILOT COMPONENTS ###

# EKF #

The EKF estimates the position of the multirotor given GPS (or MoCap if indoors) and IMU measurements.

# Controller #

A PID controller is used.

### SENSOR SUITE ###

# Motion Capture #

The [Motion Capture Wiki page](https://magiccvs.byu.edu/wiki/#!sw_guides/mocap_room_tutorial.md#VRPN_Installation) has instructions on how to include motion capture data in your ROS network.

# GPS #

# IMU #

# Barometer #


### WAYPOINT MANAGER ###

Velocity manager and commander.

Waypoints are updated through services, which are called with

`rosservice call <name_of_service> <args>`

**Note:** all waypoint services are 0 indexed.

Current waypoint services provided are as follows:

# /add_waypoint #
Adds a waypoint at a specified index. Arguments are: x y z  index

For example, the following code adds a waypoint at the x,y,z coordinate [1, 2, 3] facing along the positive y-axis as the first waypoint:

`rosservice call /add_waypoint 1 2 3 0 0`

# /remove_waypoint #
Removes a waypoint at a specified index. Argument is index.

For example, the following code removes the first waypoint:

`rosservice call /remove_waypoint 0`

# /set_waypoints_from_file #
Replaces all current waypoints with a list from a file. Currently tested with .csv and .txt files. In each row/line of the file, include the waypoint x, y, z, and orientation (separated with commas in a .txt file). The argument is the path to the file.

For example, the following code sets the waypoints from a .csv:

`rosservice call /set_waypoints_from_file /<path>/<to>/<file>/waypoints.csv`

# /list_waypoints #
Lists the current waypoints in order from the first. No arguments.

# /clear_waypoints #
Removes all waypoints. Because the multirotor needs at least 1 waypoint to be controlled, the multirotor's position when /clear_waypoints is called is set as the waypoint.


### TO DO: ###
 1. Improve GPS estimation performance using Ublox GPS
 2. Support additional altitude sensor support (laser/sonar)
 3. Heading calibration routine (magnetometer)
 4. Include wind estimation
 5. Add simpler way to add new waypoints
 6. Get states_plotter.py and commands_plotter.py
 7. Develop procedure for defining quadrotor physical parameters.  (add to wiki)
