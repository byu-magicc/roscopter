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
  bool realtime = false;
  bool verbose = false;
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
  cout << "\n";
  double bag_elapsed = 0;
  double system_elapsed = 0;
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
  bag_elapsed = (bag_end - bag_start).toSec();
  print_progress(1.0, bag_elapsed / system_elapsed);
  cout << endl;
}
