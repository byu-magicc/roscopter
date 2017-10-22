ROScopter
=========

This repository contains the ROS packages associated with flying a multirotor on the [ROSflight](rosflight.org) autopilot stack. The `roscopter` ROS package contains a high-level multirotor autopilot while the `roscopter_sim` package is used for simulation. 

The `roscopter` autopilot has been built with reference to Dr. Randy Beard's [Quadrotor Dynamics and Control](http://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub) notes, so as to allow anyone to easily understand, modify and use the code. The framework developed in the aforementioned reference closely resembles the fixed-wing framework developed in *Small Unmanned Aircraft* by Beard and McLain. This framework is inherently modular and extensively documented so as to aid the user in understanding and extending for personal use.

## Getting Started ##

Once you have cloned this repo and the [ROSflight_plugins](github.com/byu-magicc/rosflight_plugins) repo into `catkin_ws/src`, you can run the Gazebo simulator with

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter_sim multirotor.launch
```

## Core Autopilot Components ##

### MEKF ###

A Multiplicative Extended Kalman Filter (MEKF) is used for state estimation using ROSflight attitude estimates as measurements. See *Derivation of the Relative Multiplicative Kalman Filter* by David Wheeler and Daniel Koch for more information.

### Controller ###

A PID controller is used.

### Waypoint Manager ###

Velocity manager and commander.