#ifndef ROBOT_LINK_H_INCLUDED
#define ROBOT_LINK_H_INCLUDED

//headers in ROS
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

//headers in tf2
#include <tf2_ros/transform_listener.h>

//headers in urdf parser
#include <kdl_parser/kdl_parser.hpp>

class RobotLink
{
public:
  RobotLink(KDL::Vector cog_point,double mass,std::string frame);
  ~RobotLink();
  geometry_msgs::PointStamped getCogPointStamped(){return this->cog_point;};
  inline double getMass(){return this->mass;};
private:
  geometry_msgs::PointStamped cog_point;
  double mass;
};
#endif