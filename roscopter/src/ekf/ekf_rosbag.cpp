#include <iostream>
#include <stdio.h>
#include <vector>
#include <unistd.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "ekf/ekf_ros.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;

void print_progress(double progress, double rate)
{
  int total_width = 100;
  int barwidth = 70;
  cout << "[";
  int pos = barwidth * progress;
  for (int i = 0; i < barwidth; ++i)
    if (i < pos) std::cout << "=";
    else if (i == pos) std::cout << ">";
    else std::cout << " ";
  cout << "]  ";
  printf("%.3f %%\t", progress*100.0);
  printf("(%.3fx)       \r", rate);
  cout.flush();
}

int main(int argc, char * argv[])
{
  string bag_filename = "";
  string imu_topic = "";
  string seen_imu_topic = "";
  string nav_truth_topic = "";
  bool realtime = false;
  bool verbose = false;
  bool log_nav_truth = false;
  double start_time = 0;
  double duration = INFINITY;
  for (int i = 0; i < argc; i++)
  {
    string arg = argv[i];
    if (arg == "-h" || argc == 1 || arg == "--help")
    {
      cout << "USAGE: vi_ekf_rosbag [options]" << "\n\n";
      cout << "Options:\n";
      cout << "\t -h, --help\tShow this help message and exit\n";
      cout << "\t -f FILENAME\tBagfile to parse\n";
      cout << "\t -s START_TIME\tstart time of bag (seconds)\n";
      cout << "\t -r \t\tRun bag in real time\n";
      cout << "\t -u DURATION\tduration to run bag (seconds)\n";
      cout << "\t -v Show Verbose Output\n";
      cout << "\t -t Dump nav truth measurements (ie: topic of type nav_msgs/Odometry) into a log file\n";
      cout << "\t -nav Odometry topic to use for nav truth\n";
      cout << "\t -imu IMU topic to use\n";
      cout << endl;
      return 0;
    }
    else if (arg == "-imu")
    {
      if (i + 1 >= argc)
      {
        cout << "Please supply IMU topic" << endl;
        return 0;
      }
      imu_topic = argv[++i];
    }
    else if (arg == "-nav")
    {
      if (i + 1 >= argc)
      {
        cout << "Please supply Nav topic" << endl;
        return 0;
      }
      nav_truth_topic = argv[++i];
    }
    else if (arg == "-f")
    {
      if (i + 1 >= argc)
      {
        cout << "Please supply bag filename" << endl;
        return 0;
      }
      bag_filename = argv[++i];
    }
    else if (arg == "-s")
    {
      if (i + 1 >= argc)
      {
        cout << "Please specify start time" << endl;
        return 0;
      }
      start_time = atof(argv[++i]);
    }
    else if (arg == "-r")
    {
      realtime = true;
    }
    else if (arg == "-u")
    {
      if (i + 1 >= argc)
      {
        cout << "Please specify duration" << endl;
        return 0;
      }
      duration = atof(argv[++i]);
    }
    else if (arg == "-v")
    {
      verbose = true;
    }
    else if (arg == "-t")
    {
      log_nav_truth = true;
    }
    else if (i == 0)
    {
      continue;
    }
  }
  
  if (bag_filename.empty())
  {
    cout << "Please Specify bag file" << endl;
  }

  ros::init(argc, argv, "ekf_rosbag");

  rosbag::Bag bag;
  try
  {
    bag.open(bag_filename.c_str(), rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
    ROS_ERROR("unable to load rosbag %s, %s", bag_filename.c_str(), e.what());
    return -1;
  }
  rosbag::View view(bag);
  
  // Get list of topics and print to screen - https://answers.ros.org/question/39345/rosbag-info-in-c/
  if (verbose)
  {
    vector<const rosbag::ConnectionInfo *> connections = view.getConnections();
    vector<string> topics;
    vector<string> types;
    cout << "\nloaded bagfile: " << bag_filename << "\n===================================\n";
    cout << "Topics\t\tTypes\n----------------------------\n\n" << endl;
    foreach(const rosbag::ConnectionInfo *info, connections) {
      topics.push_back(info->topic);
      types.push_back(info->datatype);
      cout << info->topic << "\t\t" << info->datatype << endl;
    }
  }
  
  // Figure out the end time of the bag
  double end_time = start_time + duration;
  end_time = (end_time < view.getEndTime().toSec() - view.getBeginTime().toSec()) ? end_time : view.getEndTime().toSec() - view.getBeginTime().toSec();
  if (verbose)
    cout << "Playing bag from: = " << start_time << "s to: " << end_time << "s" << endl;
  
  // Create the VIEKF_ROS object
  roscopter::EKF_ROS node;
  
  // Get some time variables
  ros::Time system_start = ros::Time::now();
  ros::Time last_print = ros::Time(0);
  ros::Time bag_start = view.getBeginTime() + ros::Duration(start_time);
  ros::Time bag_end = view.getBeginTime() + ros::Duration(end_time);
  double bag_elapsed;
  double system_elapsed;
  cout << "\n";
  foreach (rosbag::MessageInstance const m, view)
  {
    // break on Ctrl+C
    if (!ros::ok())
      break;   
    
    // skip messages before start time
    if (m.getTime() < bag_start)
      continue;    
    
    // End bag after duration has passed
    if (m.getTime() > bag_end)
      break;

    // Run at realtime    
    if (realtime)
    {
      while (ros::Time::now() - system_start < (m.getTime() - bag_start))
      {
        usleep(1000);
      }
    }
    
    // Print status at 30 Hz
    ros::Time now = ros::Time::now();
    if (now - last_print > ros::Duration(0.03333))
    {
      bag_elapsed = (m.getTime() - bag_start).toSec();
      system_elapsed = (now - system_start).toSec();
      print_progress(bag_elapsed / (bag_end - bag_start).toSec(), bag_elapsed / system_elapsed);
      last_print = now;
    }
    
    
    /// Call all the callbacks
    
    // Cast datatype into proper format and call the appropriate callback
    string datatype = m.getDataType();
    
    if (datatype.compare("sensor_msgs/Imu") == 0)
    {
      if (!imu_topic.empty() && imu_topic.compare(m.getTopic()))
        continue;
      const sensor_msgs::ImuConstPtr imu(m.instantiate<sensor_msgs::Imu>());
      node.imu_callback(imu);

      if (!seen_imu_topic.empty() && seen_imu_topic.compare(m.getTopic()))
        ROS_WARN_ONCE("Subscribed to Two IMU messages, use the -imu argument to specify IMU topic");

      seen_imu_topic = m.getTopic();
    }

    else if (log_nav_truth && datatype.compare("nav_msgs/Odometry") == 0)
    {
      if (nav_truth_topic.empty() || nav_truth_topic.compare(m.getTopic()) == 0)
      {
        const nav_msgs::OdometryConstPtr odom(m.instantiate<nav_msgs::Odometry>());
        node.nav_truth_callback(odom);
        if (nav_truth_topic.empty())
        {
          // if this is the first time we're handling an odom message, and the
          // topic wasn't manually specified, assume we're tracking this first topic
          nav_truth_topic = m.getTopic();
        }
      } else {
        ROS_WARN_ONCE("It appears there are two different Odometry topics which could be used for Nav Truth. Use the -nav argument to specify which topic to use");
      }
    }

    else if (datatype.compare("inertial_sense/GPS") == 0)
    {
      const inertial_sense::GPSConstPtr gps(m.instantiate<inertial_sense::GPS>());
      node.gps_callback(gps);
    }
    
    else if (datatype.compare("geometry_msgs/PoseStamped") == 0)
    {
      const geometry_msgs::PoseStampedConstPtr pose(m.instantiate<geometry_msgs::PoseStamped>());      
      node.pose_truth_callback(pose);
    }
    
    else if (datatype.compare("geometry_msgs/TransformStamped") == 0)
    {
      const geometry_msgs::TransformStampedConstPtr pose(m.instantiate<geometry_msgs::TransformStamped>());
      node.transform_truth_callback(pose);
    }
  }
  ros::Time now = ros::Time::now();
  bag_elapsed = (bag_end - bag_start).toSec();
  system_elapsed = (now - system_start).toSec();
  print_progress(bag_elapsed / (bag_end - bag_start).toSec(), bag_elapsed / system_elapsed);
  cout << endl;
}
