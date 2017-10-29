#include <iostream>
#include <ros/ros.h>

#include <tethered_drone/image_processing.hpp>

using namespace tethered_drone;

// Standard C++ entry point
int main(int argc, char **argv) {

  // Announce this program to the ROS master
  ros::init(argc, argv, "imageProcessor");
  ros::NodeHandle nh; 
  ros::NodeHandle private_nh("~");

  ImageProcesser impro(nh, private_nh);  
  impro.initProcesser(private_nh);

  ros::spin();
  return 0;
}