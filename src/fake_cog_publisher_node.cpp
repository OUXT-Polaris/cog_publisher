//headers for ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fake_com_publisher");
  ros::NodeHandle nh;
  int publish_rate;
  std::string publish_frame;
  double total_mass;
  nh.param<int>(ros::this_node::getName()+"/publish_rate", publish_rate, 50);
  nh.param<std::string>(ros::this_node::getName()+"/publish_frame", publish_frame, "base_link");
  nh.param<double>(ros::this_node::getName()+"/total_mass", total_mass, 100);
  ros::Rate rate(publish_rate);
  ros::Publisher cog_robot_pub = nh.advertise<geometry_msgs::PointStamped>("/cog/robot", 1);
  ros::Publisher total_mass_pub = nh.advertise<std_msgs::Float64>("/cog/total_mass", 1);
  while(ros::ok())
  {

    rate.sleep();
  }
  return 0;
}