#include "tethered_drone/waypoint_flyer.hpp"

#include <iostream>

namespace tethered_drone {

WaypointFlyer::WaypointFlyer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      planning_complete_(false),
      frame_id_(kDefaultFrameID),
      latch_topic_(kDefaultLatchTopic),
      publish_plan_points_on_planning_complete_(
          kDefaultPublishPlanPointsOnPlanningComplete),
      publish_visualization_on_planning_complete_(
          kDefaultPublishVisualizationOnPlanningComplete) {
  // Initial interaction with ROS
  subscribeToTopics();
  getParametersFromRos();
  advertiseTopics();
}

void WaypointFlyer::subscribeToTopics() {
  //
}

void WaypointFlyer::advertiseTopics() {
  // Advertising the visualization messages
  marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "plan_markers", 1, latch_topic_);
  trajectory_points_pub_ = nh_private_.advertise<geometry_msgs::PoseArray>(
      "trajectory_points", 1, latch_topic_);
  // Services for performing publishing and visualization
  publish_all_srv_ = nh_private_.advertiseService(
      "publish_all", &WaypointFlyer::publishAllCallback, this);
  publish_visualization_srv_ = nh_private_.advertiseService(
      "publish_visualization", &WaypointFlyer::publishVisualizationCallback,
      this);
  publish_plan_points_srv_ = nh_private_.advertiseService(
      "publish_plan_points", &WaypointFlyer::publishTrajectoryPointsCallback,
      this);
}

void WaypointFlyer::getParametersFromRos() {
  // Control parameters
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("latch_topic", latch_topic_, latch_topic_);
  nh_private_.param("publish_plan_points_on_planning_complete",
                    publish_plan_points_on_planning_complete_,
                    publish_plan_points_on_planning_complete_);
  nh_private_.param("publish_visualization_on_planning_complete",
                    publish_visualization_on_planning_complete_,
                    publish_visualization_on_planning_complete_);

}

void WaypointFlyer::run() {
  // Making the plan
  loadWaypoints();

  // Publishing the plan
  if (publish_plan_points_on_planning_complete_) {
    publishPlan();
  }
  // Publishing the visualization if requested
  if (publish_visualization_on_planning_complete_) {
    publishVisualization();
  }

}

void WaypointFlyer::loadWaypoints() {
  ROS_INFO("Loading the plan");

  // Getting the waypoints in XML format
  XmlRpc::XmlRpcValue waypoints_xml;
  CHECK(nh_private_.getParam("waypoints", waypoints_xml))
      << "No vertices specified to parameter server (parameter "
         "\"waypoints\").";

  // Converting this to a vector of Eigen::Vector3d
  StdVector3d waypoints;
  xmlVerticiesToStdVector3d(waypoints_xml, &waypoints);

  // Debug output
  ROS_INFO("Loaded vertices");
  for (size_t i = 0; i < waypoints.size(); i++) {
    ROS_INFO_STREAM("waypoints[" << i << "]: " << waypoints[i].transpose());
  }

  // Waypoints list
  const size_t num_waypoints = waypoints.size();
  trajectory_points_.clear();
  trajectory_points_.resize(num_waypoints);
  for (size_t i = 0; i < waypoints.size(); i++) {
    trajectory_points_[i].position_W = waypoints[i];
  }

  // Indicating that the plan is complete
  planning_complete_ = true;
}

bool WaypointFlyer::publishPlan() {
  if (!planning_complete_) {
    ROS_WARN("Cannot send plan message because plan hasn't been made yet.");
    return false;
  }
  ROS_INFO("Sending plan messages");

  // Converting to a ros message
  geometry_msgs::PoseArray trajectory_points_pose_array;
  std::string frame_id = frame_id_;
  mav_coverage_planning::poseArrayMsgFromEigenTrajectoryPointVector(
      trajectory_points_, frame_id, &trajectory_points_pose_array);

  // Publishing
  trajectory_points_pub_.publish(trajectory_points_pose_array);

  // Success
  return true;
}

bool WaypointFlyer::publishVisualization() {
  if (!planning_complete_) {
    ROS_WARN(
        "Cannot send visualization message because plan hasn't been made yet.");
    return false;
  }
  ROS_INFO("Sending visualization messages");

  // Creating the marker array
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker path_points, path_line_strips;
  createMarkers(trajectory_points_, &path_points, &path_line_strips);
  markers.markers.push_back(path_points);
  markers.markers.push_back(path_line_strips);

  // Publishing
  marker_pub_.publish(markers);

  // Success
  return true;
}

void WaypointFlyer::createMarkers(
    const mav_msgs::EigenTrajectoryPointVector& vertices,
    const std::string& frame_id, const std::string& ns,
    const mav_visualization::Color& points_color,
    const mav_visualization::Color& lines_color,
    visualization_msgs::Marker* points,
    visualization_msgs::Marker* line_strip) {
  CHECK_NOTNULL(points);
  CHECK_NOTNULL(line_strip);
  points->points.clear();
  line_strip->points.clear();

  points->header.frame_id = line_strip->header.frame_id = frame_id;
  points->header.stamp = line_strip->header.stamp = ros::Time::now();
  points->ns = line_strip->ns = ns;
  points->action = line_strip->action = visualization_msgs::Marker::ADD;
  points->pose.orientation.w = line_strip->pose.orientation.w = 1.0;

  points->id = 0;
  line_strip->id = 1;

  points->type = visualization_msgs::Marker::POINTS;
  line_strip->type = visualization_msgs::Marker::LINE_STRIP;

  points->scale.x = 0.2;
  points->scale.x = 0.2;
  line_strip->scale.x = 0.1;

  points->color = points_color;
  line_strip->color = lines_color;

  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::Point p;
    p.x = vertices[i].position_W.x();
    p.y = vertices[i].position_W.y();
    p.z = vertices[i].position_W.z();

    points->points.push_back(p);
    line_strip->points.push_back(p);
  }
}

void WaypointFlyer::createMarkers(
    const mav_msgs::EigenTrajectoryPointVector& vertices,
    visualization_msgs::Marker* points,
    visualization_msgs::Marker* line_strip) {
  CHECK_NOTNULL(points);
  CHECK_NOTNULL(line_strip);
  createMarkers(vertices, frame_id_, "waypoints_and_strip",
                mav_visualization::Color::Red(),
                mav_visualization::Color::Green(), points, line_strip);
}

// Converts a xml matrix to a vector of points
void WaypointFlyer::xmlVerticiesToStdVector3d(XmlRpc::XmlRpcValue& vertices_xml,
                                              StdVector3d* vertices_ptr) {
  // Checks
  CHECK_NOTNULL(vertices_ptr);
  vertices_ptr->clear();
  vertices_ptr->reserve(vertices_xml.size());
  // Looping over rows of specified waypoints and adding to output vector.
  for (size_t i = 0; i < vertices_xml.size(); i++) {
    size_t num_cols = vertices_xml[i].size();
    CHECK_EQ(num_cols, 3) << "Vertex not 3D.";
    double x = vertices_xml[i][0];
    double y = vertices_xml[i][1];
    double z = vertices_xml[i][2];
    Eigen::Vector3d vertex(x, y, z);
    vertices_ptr->push_back(vertex);
  }
}

bool WaypointFlyer::publishAllCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  bool success_publish_trajectory = publishPlan();
  bool success_publish_visualization = publishVisualization();
  return (success_publish_trajectory && success_publish_visualization);
}

bool WaypointFlyer::publishVisualizationCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishVisualization();
}

bool WaypointFlyer::publishTrajectoryPointsCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishPlan();
}


}  // namespace tethered_drone