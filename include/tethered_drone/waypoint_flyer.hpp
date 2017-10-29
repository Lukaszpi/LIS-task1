#ifndef TETHERED_DRONE_WAYPOINT_FLYER
#define TETHERED_DRONE_WAYPOINT_FLYER

#include <vector>

#include <ros/ros.h>

#include <Eigen/Geometry>

#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Empty.h>

#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_visualization/helpers.h>
#include <visualization_msgs/Marker.h>

#include <mav_coverage_planning/ros_interface.h>

namespace tethered_drone {

// Convenience typedef
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
    StdVector3d;

// Parameter defaults
constexpr bool kDefaultLatchTopic = false;
const std::string kDefaultFrameID = "world";
constexpr bool kDefaultPublishPlanPointsOnPlanningComplete = true;
constexpr bool kDefaultPublishVisualizationOnPlanningComplete = true;

class WaypointFlyer {
 public:
  WaypointFlyer(ros::NodeHandle nh, ros::NodeHandle nh_private);

  // Runs the function
  void run();

  // Loads the waypoints from the ROS param server
  void loadWaypoints();

  // Publishes the plan visualization
  bool publishPlan();
  bool publishVisualization();

 private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // Service callbacks
  bool publishAllCallback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response);
  bool publishVisualizationCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);
  bool publishTrajectoryPointsCallback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response);


  void createMarkers(const mav_msgs::EigenTrajectoryPointVector& vertices,
                     visualization_msgs::Marker* points,
                     visualization_msgs::Marker* line_strip);
  void createMarkers(const mav_msgs::EigenTrajectoryPointVector& vertices,
                     const std::string& frame_id, const std::string& ns,
                     const mav_visualization::Color& points_color,
                     const mav_visualization::Color& lines_color,
                     visualization_msgs::Marker* points,
                     visualization_msgs::Marker* line_strip);

  void xmlVerticiesToStdVector3d(XmlRpc::XmlRpcValue& vertices_xml,
                                 StdVector3d* vertices_ptr);

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Services
  ros::Publisher marker_pub_;
  ros::Publisher trajectory_points_pub_;
  ros::ServiceServer publish_visualization_srv_;
  ros::ServiceServer publish_plan_points_srv_;
  ros::ServiceServer publish_all_srv_;

  // The plan
  mav_msgs::EigenTrajectoryPointVector trajectory_points_;

  // A flag indicating the planning status
  bool planning_complete_;

  // Control flags
  bool publish_plan_points_on_planning_complete_;
  bool publish_visualization_on_planning_complete_;

  // Parameters
  std::string frame_id_;
  bool latch_topic_;

};

}  // namespace tethered_drone

#endif