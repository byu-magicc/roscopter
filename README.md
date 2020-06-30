ROScopter
=========

This repository contains the ROS packages associated with flying a multirotor on the [ROSflight](http://rosflight.org) autopilot stack. The `roscopter` ROS package contains a high-level multirotor autopilot while the `roscopter_sim` package is used for simulation.

View Wiki for detailed step by step setup instructions

https://github.com/byu-magicc/roscopter/wiki

## Running Simulation ##

# Simplified Dynamics Model

Clone this repo, the [ROSflight_plugins](https://github.com/byu-magicc/rosflight_plugins) repo, and the [ROSflight](https://github.com/rosflight/rosflight.git) repo into `catkin_ws/src`, you can run the Gazebo simulator with

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter_sim multirotor.launch
```
This simulation does not simulate the rosflight firmware and does not require a joystick. It has the same API as the SIL stack, and can be useful for developing high-level algorithms, however it makes some simplifying assumptions in the dynamics which reduces the fidelity of the simulation.

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

# TODO: #

1. Develop and document process noise std tuning methods.

2. Develop and document a tuning procedure for the multirotor. This includes tuning all the gain parameters for the flight vehicle.

3. Standardize the flight controller board. Board sensor suite needs to include an IMU, barometer and magnetometer. Current board and sensor combination is becoming obsolete.

4. Implement a trajectory follower control node. Should recieve inertial position, velocity, acceleration and heading, and output angular rate and throttle.

5. Develop and document a magnetometer calibration routine and incorporate magnetometer measurements to the estimator. This would enable accurate heading estimates at initialization and during hover flight.

6. Support an additional altitude sensor for landing. Possible options include sonar, laser, or camera.

7. Create a failsafe mode so the quadrotor can have a safe landing. Use cases include when RC connection is lost.

8. Improve flight performance in windy conditions. Most accurate solution is to incorporate wind estimation. This is possible without air sensors if you can estimate parameters of the flight vehicle such as drag and thrust coefficients.
