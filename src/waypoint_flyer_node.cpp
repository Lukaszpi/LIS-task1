#include <ros/ros.h>

#include "tethered_drone/waypoint_flyer.hpp"

// Standard C++ entry point
int main(int argc, char **argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "waypoint_flyer_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Creating the object to do the work
  tethered_drone::WaypointFlyer waypoint_flyer(nh, nh_private);
  waypoint_flyer.run();

  ros::spin();
  return 0;
}