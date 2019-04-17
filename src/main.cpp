#include <iostream>
#include <ros/ros.h>



int main(int argc, char **argv) {

  std::cout << std::fixed;
  std::cout << std::setprecision(10);

  ros::init(argc, argv, "gmsl_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("GMSL node started!");


  ros::spin();
  return 0;
}