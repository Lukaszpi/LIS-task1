#ifndef TETHERED_DRONE_IMAGEPROCESSER
#define TETHERED_DRONE_IMAGEPROCESSER

#include <string>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include "sensor_msgs/Image.h"
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace tethered_drone {

// Convenience typedef
typedef kindr::minimal::QuatTransformation Transformation;

struct TransformationStamped {
  ros::Time stamp;
  Transformation transformation;
};

//********************************************//********************************************
// class for masking the image and detecting the points
//********************************************//********************************************

class ImageProcesser {

public:
  ImageProcesser(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void processImage(const sensor_msgs::ImageConstPtr &image_message);
  void listenTransform(const geometry_msgs::TransformStampedPtr &pose_message);

  bool check_point(std::vector<cv::Point2d> points_to_match,
                   std::vector<cv::Point2d> points_to_search, double tresh);

  bool bruteForceMatcher(std::vector<cv::Point2d> found_blobs);
  bool trackedMatcher(std::vector<cv::Point2d> found_blobs);

  bool useMask_ = false;

  cv::Mat getRotation() { return curr_rot_; }
  cv::Mat getTranslation() { return curr_trans_; }

  void initProcesser(ros::NodeHandle nh_private);

  bool validatePose(cv::Mat tvec, cv::Mat rvec);

private:
  bool trackingMode_;
  int required_points_;
  bool first_time_flag_;

  bool valid_last_pose_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber image_sub_;
  ros::Subscriber transform_sub_;
  ros::Publisher result_pub_;
  ros::Publisher img_pup_; 

  cv::Mat distCoeffs_;
  cv::Mat cameraMatrix_;
  std::vector<cv::Point3d> worldPattern_;
  std::vector<cv::Point3d> selected_pattern_points_;

  cv::Mat curr_rot_;
  cv::Mat old_rot_;
  cv::Mat curr_trans_;
  cv::Mat old_trans_;
  cv_bridge::CvImagePtr curr_imgPtr_;

  cv::Mat est_rot_;
  cv::Mat est_trans_;

  // tf stuff
  std::string local_frame_name_;
  std::string global_frame_name_;
  double tf_publish_time_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  tf::TransformListener listener_;
  tf::StampedTransform transform_odom_camdown_;
  tf::StampedTransform transform_odom_pattern_;
  // Stores the current subscribed transform
  TransformationStamped transform_C_P_;

  // Publishes the resulting transform
  void publishTFTransform(const ros::TimerEvent &event);

  std::vector<cv::Point2d> detectPoints(cv_bridge::CvImagePtr cvPtr);
  void maskImage(cv_bridge::CvImagePtr imgPtr,
                 geometry_msgs::TransformStamped estimated_transform);

  void displayPoints(std::vector<cv::Point2d> image_points,
                     std::vector<cv::Point2d> image_points_set2);
};

} // namespace tethered_drone

#endif