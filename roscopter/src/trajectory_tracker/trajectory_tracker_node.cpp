#include <ros/ros.h>
#include "trajectory_tracker/trajectory_tracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_tracker_node");
    ros::NodeHandle nh;
    trajectory_tracker::TrajectoryTracker TrajectoryTrackerObject;
    ros::spin();
    return 0;
}