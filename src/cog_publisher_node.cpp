//headers in this package
#include <cog_publisher/cog_publisher.h>

//headers for ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "com_publisher");
  CogPublisher center_of_gravity_publisher = CogPublisher();
  ros::Rate rate(center_of_gravity_publisher.get_publish_rate());
  while(ros::ok())
  {
    center_of_gravity_publisher.publish();
    rate.sleep();
  }
  return 0;
}