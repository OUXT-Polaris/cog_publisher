//headers in this package
#include <robot_link.h>

//headers fot tf2
#include <tf2_ros/transform_listener.h>

//headers for urdf parser
#include <kdl_parser/kdl_parser.hpp>

robot_link::robot_link(KDL::Vector cog_point,double mass,std::string frame)
{
  this->mass = mass;
  this->cog_point.point.x = cog_point.x();
  this->cog_point.point.y = cog_point.y();
  this->cog_point.point.z = cog_point.z();
  this->cog_point.header.frame_id = frame;
}

robot_link::~robot_link()
{

}