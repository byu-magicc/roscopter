# ros_copter

This is intended to eventually be a fully-featured multirotor autopilot for ROS.  It will be built according to the method published in [Quadrotor Dynamics and Control](http://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub), so as to allow anyone to easily understand, modify and use the code.  The framework developed in the afore mentioned reference closely resembles the fixed wing framework developed in Small Unmanned Aircraft by Beard and McLain.  This framework is inherently modular and extensively documented so as to aid the user in understanding and extending for personal use.

The package is intended to be used with fcu\_io or fcu\_sim, for hardware with a naze32 or spracingf3 (or derivatives) or simulation, respectively.

It is a single ROS package, with several nodes.

# - EKF 

The ekf package contains a standard mekf, as defined mostly in the way in the reference above.  We are probably going to release a new version of the Quadrotor Dynamics and Control to resemble more closely the modeling in this node.  We are estimating position, velocity, and attitude as well as accelerometer biases, velocities are body fixed, and the accelerometer biases allow for better estimation and control long term.   The model and jacobians are defined explicitly in the doc/ekf_jacobians.pdf document

# - PID Controller

To be implemented (will be general PID control as defined in the reference above)

# - Path Planner

To be implemented (will probably use an RRT-based planner, with knowledge of the environment)

# - Path Follower

To be implemented (will use a waypoint-following technique as described in the UAS book)

# - Monocular Visual Odometry

May or may not implement.  This likely is better done by SVO, and if so, we will just make sure it integrates well with our stuff.
