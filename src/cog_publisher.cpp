//headers in this package
#include <cog_publisher.h>
#include <robot_link.h>

//headers for standard library
#include<iostream>
#include<fstream>

//headers for ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>

//headers fot tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers for urdf parser
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>

cog_publisher::cog_publisher()
{
  std::string robot_description_text;
  //get publish_freme parameter
  nh.param<std::string>(ros::this_node::getName()+"/publish_frame", publish_frame, "base_link");
  //get publish_rate parameter
  nh.param<int>(ros::this_node::getName()+"/publish_rate", publish_rate, 50);
  //get robot_description parameter
  nh.getParam("/robot_description", robot_description_text);
  KDL::Tree robot_tree;
  //parse urdf by using kdl_parser
  kdl_parser::treeFromString(robot_description_text, robot_tree);
  std::map<std::string,KDL::TreeElement> robot_segment = robot_tree.getSegments();
  //making tf listener instance
  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);
  //initialize robot total mass
  robot_total_mass = 0;
  //iterate each link and get parameters
  for(auto robot_link_link = robot_segment.begin(); robot_link_link != robot_segment.end(); ++robot_link_link)
  {
    //get link name from std::map
    std::string link_name = robot_link_link->first;
    //get link parameters
    KDL::RigidBodyInertia rigid_body_inertia = robot_link_link->second.segment.getInertia();
    //get Mass of link
    double mass = rigid_body_inertia.getMass();
    //get Center of Gravity parameters
    KDL::Vector cog_point = rigid_body_inertia.getCOG();
    robot_link link(cog_point,mass,link_name);
    links.push_back(link);
    robot_total_mass = robot_total_mass + mass;
  }
  cog_links_pub = nh.advertise<sensor_msgs::PointCloud>("/cog/links", 1);
  cog_robot_pub = nh.advertise<geometry_msgs::PointStamped>("/cog/robot", 1);
}

void cog_publisher::publish()
{
  //cog of each links
  sensor_msgs::PointCloud cog_links_msg;
  geometry_msgs::PointStamped cog_point,cog_robot;
  ros::Time now = ros::Time::now();
  cog_links_msg.header.stamp = now;
  cog_links_msg.header.frame_id = publish_frame;
  //parameters for calculate COG of robot
  double cog_robot_x = 0;
  double cog_robot_y = 0;
  double cog_robot_z = 0;
  //iterate each link parameter
  for(auto link = links.rbegin(); link != links.rend(); ++link)
  {
    //cog point of a link
    geometry_msgs::Point32 cog_link;
    //transform data from this link frame to /publish_frame(default base_link)
    geometry_msgs::TransformStamped transform;
    //get link parameters
    cog_point = link->get_cog_point_stamped();
    cog_point.header.stamp = now;
    //transform to /publish_frame(default base_link)
    try
    {
      transform = tf_buffer->lookupTransform(publish_frame, cog_point.header.frame_id, ros::Time(0));
      tf2::doTransform(cog_point, cog_point, transform);
      //convert from geometry_msgs/PointStamped to geometry_msgs/Point32
      cog_link.x = cog_point.point.x;
      cog_link.y = cog_point.point.y;
      cog_link.z = cog_point.point.z;
      cog_links_msg.points.push_back(cog_link);
      cog_robot_x = link->get_mass()*cog_link.x/robot_total_mass + cog_robot_x;
      cog_robot_y = link->get_mass()*cog_link.y/robot_total_mass + cog_robot_y;
      cog_robot_z = link->get_mass()*cog_link.z/robot_total_mass + cog_robot_z;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      return;
    }
  }
  //input robot cog data
  cog_robot.point.x = cog_robot_x;
  cog_robot.point.y = cog_robot_y;
  cog_robot.point.z = cog_robot_z;
  cog_robot.header.frame_id = publish_frame;
  cog_robot.header.stamp = now;
  //publish cog point of robot (whole body)
  cog_robot_pub.publish(cog_robot);
  //publish cog point of each link
  cog_links_pub.publish(cog_links_msg);
}

cog_publisher::~cog_publisher()
{

}