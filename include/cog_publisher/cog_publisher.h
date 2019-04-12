#ifndef COG_PUBLISHER_H_INCLUDED
#define COG_PUBLISHER_H_INCLUDED

//headers in this package
#include <cog_publisher/robot_link.h>

//headers in standard library
#include <vector>
#include<iostream>
#include<fstream>

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers in KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>

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