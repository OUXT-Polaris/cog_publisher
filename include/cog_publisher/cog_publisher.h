#ifndef COG_PUBLISHER_H_INCLUDED
#define COG_PUBLISHER_H_INCLUDED

//headers in this package
#include <cog_publisher/robot_link.h>

//headers for standard library
#include <vector>

//headers for ROS
#include <ros/ros.h>

class CogPublisher
{
public:
  CogPublisher();
  ~CogPublisher();
  //publish COG
  void publish();
  //parameter getter
  inline int getPublishRate(){return this->publish_rate;};
  inline double getRobotTotalMass(){return this->robot_total_mass;};
private:
  ros::NodeHandle nh;
  std::vector<RobotLink> links;
  //ros publisher and subscriber
  ros::Publisher cog_links_pub,cog_robot_pub,total_mass_pub;
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  //parameters
  std::string publish_frame;
  int publish_rate;
  double robot_total_mass;
};
#endif