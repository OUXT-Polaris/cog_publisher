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
  double cog_x,cog_y,cog_z;
  nh.param<int>(ros::this_node::getName()+"/publish_rate", publish_rate, 50);
  nh.param<std::string>(ros::this_node::getName()+"/publish_frame", publish_frame, "base_link");
  nh.param<double>(ros::this_node::getName()+"/total_mass", total_mass, 100);
  nh.param<double>(ros::this_node::getName()+"/cog_x", cog_x, 0);
  nh.param<double>(ros::this_node::getName()+"/cog_y", cog_y, 0);
  nh.param<double>(ros::this_node::getName()+"/cog_z", cog_z, 0);
  ros::Rate rate(publish_rate);
  ros::Publisher cog_robot_pub = nh.advertise<geometry_msgs::PointStamped>("/cog/robot", 1);
  geometry_msgs::PointStamped cog_point_msg;
  cog_point_msg.header.frame_id = publish_frame;
  cog_point_msg.point.x = cog_x;
  cog_point_msg.point.y = cog_y;
  cog_point_msg.point.z = cog_z;
  ros::Publisher total_mass_pub = nh.advertise<std_msgs::Float64>("/cog/total_mass", 1);
  std_msgs::Float64 total_mass_msg;
  total_mass_msg.data = total_mass;
  while(ros::ok())
  {
    cog_point_msg.header.stamp = ros::Time::now();
    cog_robot_pub.publish(cog_point_msg);
    total_mass_pub.publish(total_mass_msg);
    rate.sleep();
  }
  return 0;
}