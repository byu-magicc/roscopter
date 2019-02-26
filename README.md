ROScopter
=========

This repository contains the ROS packages associated with flying a multirotor on the [ROSflight](http://rosflight.org) autopilot stack. The `roscopter` ROS package contains a high-level multirotor autopilot while the `roscopter_sim` package is used for simulation. 

The `roscopter` autopilot has been built with reference to Dr. Randy Beard's [Quadrotor Dynamics and Control](http://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub) notes, so as to allow anyone to easily understand, modify and use the code. The framework developed in the aforementioned reference closely resembles the fixed-wing framework developed in *Small Unmanned Aircraft* by Beard and McLain. This framework is inherently modular and extensively documented so as to aid the user in understanding and extending for personal use.

## Running Simulation ##

### Simplified Dynamics Model

Clone this repo and the [ROSflight_plugins](github.com/byu-magicc/rosflight_plugins) repo into `catkin_ws/src`, you can run the Gazebo simulator with

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter_sim multirotor.launch
```

This simulation does not simulate the firmware and does not require a joystick. It has the same API as the SIL stack, and can be useful for developing high-level algorithms, however it makes some simplifying assumptions in the dynamics which reduces the fidelity of the simulation.

### Software In The Loop 
To run the `rosflight` SIL simulation [information](http://docs.rosflight.org/en/latest/user-guide/gazebo_simulation/) with the full roscopter stack, clone the [ROSflight](github.com/rosflight/rosflight) repo, connect a joystick or transmitter and run

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter copter_sil..launch
```

## Core Autopilot Components ##

### EKF ###

An Extended Kalman Filter (EKF) on manifold is used for state estimation using ROSflight attitude estimates as measurements. See *Derivation of the Relative Multiplicative Kalman Filter* by David Wheeler and Daniel Koch for more information.

The EKF supports measurements from a position and attitude estimator (motion capture system), altitude sensor (sonar or barometer) and IMU.  GPS support is currently being developed.  Measurement models can be added quite easily to the filter, please create an Issue if you have a measurement model you want supported and we can walk you through the steps on how to do it.

### Controller ###

A PID controller is used.

### Waypoint Manager ###

Velocity manager and commander.
