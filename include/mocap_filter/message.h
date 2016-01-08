#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

namespace mocap_filter{

enum type_t {IMU, POSE_STAMPED};


class Message{
public:
  std_msgs::Header header;
  type_t type;
  virtual geometry_msgs::Vector3 getAngularVelocity() = 0;
  virtual boost::array<double, 9ul> getAngularVelocityCovariance() = 0;
  virtual geometry_msgs::Vector3 getLinearAcceleration() = 0;
  virtual boost::array<double, 9ul> getLinearAccelerationCovariance() = 0;
  virtual geometry_msgs::Pose getPose() = 0;
};


class PoseMessage : public Message{
public:
  PoseMessage(){
    type = POSE_STAMPED;
  }

  PoseMessage(geometry_msgs::PoseStamped msg){
    header = msg.header;
    type = POSE_STAMPED;
    pose = msg.pose;
  }
  geometry_msgs::Pose getPose(){
   return pose;
  }

private:
  geometry_msgs::Pose pose;
};


class ImuMessage : public Message{
public:
  ImuMessage(){
    type = IMU;
  }
  ImuMessage(sensor_msgs::Imu msg){
    header = msg.header;
    type = IMU;
    angular_velocity = msg.angular_velocity;
    angular_velocity_covariance = msg.angular_velocity_covariance;
    linear_acceleration = msg.linear_acceleration;
    linear_acceleration_covariance = msg.linear_acceleration_covariance;
  }

  geometry_msgs::Vector3 getLinearAcceleration(){
    return linear_acceleration;
  }

  boost::array<double, 9ul> getLinearAccelerationCovariance(){
    return linear_acceleration_covariance;
  }

  geometry_msgs::Vector3 Vector3getAngularVelocity(){
    return angular_velocity;
  }

  boost::array<double, 9ul> getAngularVelocityCovariance(){
    return angular_velocity_covariance;
  }

private:
  geometry_msgs::Vector3 angular_velocity;
  boost::array<double, 9ul> angular_velocity_covariance;

  geometry_msgs::Vector3 linear_acceleration;
  boost::array<double, 9ul> linear_acceleration_covariance;
};

}
