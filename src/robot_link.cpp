//headers in this package
#include <cog_publisher/robot_link.h>

RobotLink::RobotLink(KDL::Vector cog_point,double mass,std::string frame)
{
  this->mass = mass;
  this->cog_point.point.x = cog_point.x();
  this->cog_point.point.y = cog_point.y();
  this->cog_point.point.z = cog_point.z();
  this->cog_point.header.frame_id = frame;
}

RobotLink::~RobotLink()
{

}