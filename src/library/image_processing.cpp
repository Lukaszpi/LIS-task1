#include "tethered_drone/image_processing.hpp"

#include <eigen_conversions/eigen_msg.h>
#include <kindr/minimal/quat-transformation.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using std::cout;
using std::endl;

namespace tethered_drone {

// Default values for parameters
static const std::string kDefaultLocalFrameName = "body";
static const std::string kDefaultGlobalFrameName = "odom";
constexpr double kDefaultTFPublishTime = 0.1;

/*********************************************************************
Image Processing Class
*********************************************************************/
ImageProcesser::ImageProcesser(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : curr_rot_(cv::Mat::zeros(3, 1, CV_64F)),
      curr_trans_(cv::Mat::zeros(3, 1, CV_64F)),
      est_rot_(cv::Mat::zeros(3, 1, CV_64F)),
      est_trans_(cv::Mat::zeros(3, 1, CV_64F)),
      cameraMatrix_(cv::Mat::zeros(3, 3, CV_64F)),
      distCoeffs_(cv::Mat::zeros(4, 1, CV_64F)), valid_last_pose_(false),
      nh_private_(nh_private), nh_(nh), first_time_flag_(true) {

  nh_private.getParam("/ibis/image_processer/tracking_mode", trackingMode_);
  nh_private_.getParam("/ibis/image_processer/required_points",
                       required_points_);
  nh_private_.getParam("/ibis/image_processer/use_mask", useMask_);

  transform_sub_ = nh.subscribe("/ibis/transform_estimator/pose_filtered", 10,
                                &ImageProcesser::listenTransform, this);
  image_sub_ = nh.subscribe("/ibis/chameleon3/image_mono", 10,
                            &ImageProcesser::processImage, this);

  result_pub_ = nh_private.advertise<geometry_msgs::TransformStamped>(
      "relative_pose", 10);

  img_pup_ = nh_private.advertise<sensor_msgs::Image>("debug_image", 10);

  nh_private.getParam("/ibis/image_processer/local_frame_name",
                      local_frame_name_);
  nh_private.getParam("/ibis/image_processer/global_frame_name",
                      global_frame_name_);
  nh_private.getParam("/ibis/image_processer/tf_publish_time",
                      tf_publish_time_);

  tf_timer_ = nh.createTimer(ros::Duration(tf_publish_time_),
                             &ImageProcesser::publishTFTransform, this);
}

void ImageProcesser::processImage(
    const sensor_msgs::ImageConstPtr &image_message) {

  cv_bridge::CvImagePtr cvPtr;

  cvPtr =
      cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::BGR8);

  std::vector<cv::Point2d> found_blobs;
  found_blobs.clear();
  found_blobs = detectPoints(cvPtr);

  displayPoints(found_blobs, found_blobs);

  if (found_blobs.size() < 6) {
    ROS_INFO("Not enough points for Pose Estimation");
    valid_last_pose_ = false;

  } else if (!valid_last_pose_) {

    valid_last_pose_ = bruteForceMatcher(found_blobs);

  } else if (trackingMode_) {

    valid_last_pose_ = trackedMatcher(found_blobs);

    if (!valid_last_pose_) {
      valid_last_pose_ = bruteForceMatcher(found_blobs);
    }
  } else {
    valid_last_pose_ = bruteForceMatcher(found_blobs);
  }

  if (valid_last_pose_) {

    //...........................................................................
    // For publishing the geometry msgs

    geometry_msgs::TransformStamped calculated_relative_pose_msgs;

    calculated_relative_pose_msgs.header = image_message->header;

    cv::Mat rotToQuat(3, 3, CV_64F, cv::Scalar(0));
    cv::Rodrigues(curr_rot_, rotToQuat);

    Eigen::Matrix3d eigen_rotToQuat;
    eigen_rotToQuat << rotToQuat.at<double>(0, 0), rotToQuat.at<double>(0, 1),
        rotToQuat.at<double>(0, 2), rotToQuat.at<double>(1, 0),
        rotToQuat.at<double>(1, 1), rotToQuat.at<double>(1, 2),
        rotToQuat.at<double>(2, 0), rotToQuat.at<double>(2, 1),
        rotToQuat.at<double>(2, 2);

    Eigen::Quaterniond quat_eigen(eigen_rotToQuat);
    Eigen::Vector3d vec_eigen(curr_trans_.at<double>(0, 0),
                              curr_trans_.at<double>(1, 0),
                              curr_trans_.at<double>(2, 0));

    tf::quaternionEigenToMsg(quat_eigen,
                             calculated_relative_pose_msgs.transform.rotation);

    calculated_relative_pose_msgs.transform.translation.x =
        (curr_trans_.at<double>(0, 0));
    calculated_relative_pose_msgs.transform.translation.y =
        (curr_trans_.at<double>(1, 0));
    calculated_relative_pose_msgs.transform.translation.z =
        (curr_trans_.at<double>(2, 0));

    calculated_relative_pose_msgs.child_frame_id = "led_patter_direct";
    calculated_relative_pose_msgs.header.frame_id = "cam_down";

    result_pub_.publish(calculated_relative_pose_msgs);

    //...........................................................................
    // For publishing the tf
    transform_C_P_.transformation = Transformation(quat_eigen, vec_eigen);

    tf::Transform tf_transform;
    tf::transformKindrToTF(transform_C_P_.transformation, &tf_transform);

    listener_.lookupTransform(global_frame_name_, "cam_down", ros::Time(0),
                              transform_odom_camdown_);

    transform_C_P_.stamp = ros::Time::now();

    Transformation T_O_CD_kindr;
    transformTFToKindr(transform_odom_camdown_, &T_O_CD_kindr);

    Transformation T_O_P_kindr = T_O_CD_kindr * transform_C_P_.transformation;

    Eigen::Vector3d euler_angles =
        T_O_P_kindr.getRotationMatrix().eulerAngles(2, 1, 0);
    double yaw_angle = euler_angles(0);

    Eigen::Quaterniond quat_gravity_aligned_eigen =
        Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw_angle * M_PI, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d p_O_P_eigen = T_O_P_kindr.getPosition();

    Transformation T_O_P_gravity_aligned_kindr =
        Transformation(quat_gravity_aligned_eigen, p_O_P_eigen);

    tf::Transform T_O_P_gravity_aligned_tf;
    tf::transformKindrToTF(T_O_P_gravity_aligned_kindr,
                           &T_O_P_gravity_aligned_tf);

    tf::Transform T_O_P_tf;
    tf::transformKindrToTF(T_O_P_kindr, &T_O_P_tf);

    transform_odom_pattern_ =
        tf::StampedTransform(T_O_P_gravity_aligned_tf, transform_C_P_.stamp,
                             global_frame_name_, local_frame_name_);

    if (first_time_flag_ = true) {
      first_time_flag_ = false;
    }
  }
}

//***********************************************************************************************
// Save the filtered pose
//***********************************************************************************************
void ImageProcesser::listenTransform(
    const geometry_msgs::TransformStampedPtr &pose_message) {

  est_trans_.at<double>(0, 0) = pose_message->transform.translation.x;
  est_trans_.at<double>(1, 0) = pose_message->transform.translation.y;
  est_trans_.at<double>(2, 0) = pose_message->transform.translation.z;

  Eigen::Quaterniond quat_current_rotation;
  tf::quaternionMsgToEigen(pose_message->transform.rotation,
                           quat_current_rotation);

  Eigen::Matrix3d mat_current_rotation;
  mat_current_rotation = quat_current_rotation.toRotationMatrix();

  cv::Mat3d cvMat_est_rot;
  cv::eigen2cv(mat_current_rotation, cvMat_est_rot);

  cv::Rodrigues(cvMat_est_rot, est_rot_);
}

//***********************************************************************************************
// Initalize parameters
//***********************************************************************************************
void ImageProcesser::initProcesser(ros::NodeHandle nh_private) {

  double fx, fy, cx, cy; // intrinsics

  double k1, k2, p1, p2; // distortion coeffs

  nh_private.getParam("/ibis/image_processer/chameleon3/distortion_coeffs/k1",
                      k1);
  nh_private.getParam("/ibis/image_processer/chameleon3/distortion_coeffs/k2",
                      k2);
  nh_private.getParam("/ibis/image_processer/chameleon3/distortion_coeffs/p1",
                      p1);
  nh_private.getParam("/ibis/image_processer/chameleon3/distortion_coeffs/p2",
                      p2);

  nh_private.getParam("/ibis/image_processer/chameleon3/intrinsics/fx", fx);
  nh_private.getParam("/ibis/image_processer/chameleon3/intrinsics/fy", fy);
  nh_private.getParam("/ibis/image_processer/chameleon3/intrinsics/cx", cx);
  nh_private.getParam("/ibis/image_processer/chameleon3/intrinsics/cy", cy);

  cameraMatrix_.at<double>(0, 0) = fx;
  cameraMatrix_.at<double>(0, 2) = cx;
  cameraMatrix_.at<double>(1, 1) = fy;
  cameraMatrix_.at<double>(1, 2) = cy;
  cameraMatrix_.at<double>(2, 2) = 1;

  distCoeffs_.at<double>(0, 0) = k1;
  distCoeffs_.at<double>(0, 1) = k2;
  distCoeffs_.at<double>(0, 2) = p1;
  distCoeffs_.at<double>(0, 3) = p2;

  worldPattern_.clear();
  worldPattern_.push_back(cv::Point3d(-0.15, 0.0, 0.0));
  worldPattern_.push_back(cv::Point3d(0.15, 0.0, 0.0));
  worldPattern_.push_back(cv::Point3d(0.3, 0.0, 0.0));
  worldPattern_.push_back(cv::Point3d(0.0, -0.3, 0.0));
  worldPattern_.push_back(cv::Point3d(0.0, -0.15, 0.0));
  worldPattern_.push_back(cv::Point3d(0.0, 0.15, 0.0));
  worldPattern_.push_back(cv::Point3d(0.0, 0.3, 0.0));
  worldPattern_.push_back(cv::Point3d(0.0, 0.45, 0.0));
  worldPattern_.push_back(cv::Point3d(-0.3, 0.25, 0.0));
  worldPattern_.push_back(cv::Point3d(0.2, -0.1, 0.0));

  // preselected pattern points for brute-force matching
  selected_pattern_points_.clear();
  selected_pattern_points_.push_back(cv::Point3d(0.0, 0.15, 0.0));
  selected_pattern_points_.push_back(cv::Point3d(0.0, -0.15, 0.0));
  selected_pattern_points_.push_back(cv::Point3d(0.15, 0.0, 0.0));
  selected_pattern_points_.push_back(cv::Point3d(-0.15, 0.0, 0.0));
}

//***********************************************************************************************
// Function to check for existing points
//***********************************************************************************************
bool ImageProcesser::check_point(std::vector<cv::Point2d> points_to_match,
                                 std::vector<cv::Point2d> points_to_search,
                                 double tresh) {

  int counter = 0;
  double dist;

  for (int i = 0; i < points_to_search.size(); ++i) {
    for (int j = 0; j < points_to_match.size(); ++j) {
      dist = sqrt(pow(points_to_match.at(j).x - points_to_search.at(i).x, 2) +
                  pow(points_to_match.at(j).y - points_to_search.at(i).y, 2));

      if (dist < tresh) {
        counter++;
        break;
      }
    }
  }

  if (counter > required_points_) {
    return true;
  }
  return false;
}

//***********************************************************************************************
// Debugging functions for points to images
//***********************************************************************************************
void ImageProcesser::displayPoints(std::vector<cv::Point2d> image_points,
                                   std::vector<cv::Point2d> image_points_set2) {
  cv::Mat camera_picture(480, 752, CV_8UC1, cv::Scalar(0));

  for (int i = 0; i < image_points.size(); i++) {
    cv::circle(camera_picture, image_points.at(i), 10,
               cv::Scalar(255, 255, 255), 1);
  }
  for (int i = 0; i < image_points_set2.size(); i++) {
    cv::circle(camera_picture, image_points_set2.at(i), 3,
               cv::Scalar(255, 255, 255), -1);
  }

  cv_bridge::CvImage out_img_msg;

  out_img_msg.encoding = sensor_msgs::image_encodings::MONO8;
  out_img_msg.image = camera_picture;

  img_pup_.publish(out_img_msg);
}

//***********************************************************************************************
// Function to find blobs
//***********************************************************************************************
std::vector<cv::Point2d>
ImageProcesser::detectPoints(cv_bridge::CvImagePtr cvPtr) {

  if (useMask_) {

    int rows = (cvPtr->image).rows;
    int cols = (cvPtr->image).cols;
    cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8U);

    std::vector<cv::Point2d> predicted_imagepoint_location;

    cv::Mat tvec = getTranslation();
    cv::Mat rvec = getRotation();

    cv::projectPoints(worldPattern_, rvec, tvec, cameraMatrix_, distCoeffs_,
                      predicted_imagepoint_location);

    for (int i = 0; i < predicted_imagepoint_location.size(); i++) {
      cv::circle(mask, predicted_imagepoint_location.at(i), 50, (255, 255, 255),
                 -1);
    }

    cv_bridge::CvImagePtr maskedImage;
    maskedImage.reset(new cv_bridge::CvImage);

    (cvPtr->image).copyTo(maskedImage->image, mask);

    (maskedImage->image).copyTo(cvPtr->image);
  }

  cv::SimpleBlobDetector::Params params;

  params.thresholdStep = 5;
  params.minThreshold = 100;
  params.maxThreshold = 255;

  params.minDistBetweenBlobs = 15.0f;

  params.filterByInertia = false;
  params.minInertiaRatio = 0.6;
  params.maxInertiaRatio = 1;

  params.filterByConvexity = false;
  params.minConvexity = 0.8;
  params.maxConvexity = 1;

  params.filterByColor = false;
  params.blobColor = 255;

  params.filterByCircularity = false;
  params.minCircularity = 0.8;
  params.maxCircularity = 1;

  params.filterByArea = true;
  params.minArea = 0.05f;
  params.maxArea = 2000.0f;

  cv::SimpleBlobDetector blob_detector(params);
  std::vector<cv::KeyPoint> keyPoints;

  blob_detector.detect(cvPtr->image, keyPoints);

  std::vector<cv::Point2d> found_blobs;
  for (int i = 0; i < keyPoints.size(); i++) {
    found_blobs.push_back(keyPoints[i].pt);
  }

  return found_blobs;
}

//***********************************************************************************************
// Matches input points by comparing every point with every other
//***********************************************************************************************
bool ImageProcesser::bruteForceMatcher(std::vector<cv::Point2d> found_blobs) {

  bool found_pose = false;
  cv::Mat rvec_pnp(3, 1, CV_64F, 0.0f);
  cv::Mat tvec_pnp(3, 1, CV_64F, 0.0f);
  std::vector<cv::Point2d> selected_image_points;
  std::vector<cv::Point2d> expected_pattern_points;

  for (int p1 = 0; p1 < found_blobs.size(); p1++) {
    if (found_pose) {
      break;
    }
    for (int p2 = 0; p2 < found_blobs.size(); p2++) {
      if (found_pose) {
        break;
      }
      if (p1 == p2) {
        continue;
      }
      for (int p3 = 0; p3 < found_blobs.size(); p3++) {
        if (found_pose) {
          break;
        }
        if (p1 == p3 || p2 == p3) {
          continue;
        }

        for (int p4 = 0; p4 < found_blobs.size(); p4++) {
          if (found_pose) {
            break;
          }
          if (p1 == p4 || p2 == p4 || p3 == p4) {
            continue;
          }

          selected_image_points.clear();
          selected_image_points.push_back(found_blobs.at(p1));
          selected_image_points.push_back(found_blobs.at(p2));
          selected_image_points.push_back(found_blobs.at(p3));
          selected_image_points.push_back(found_blobs.at(p4));

          solvePnP(selected_pattern_points_, selected_image_points,
                   cameraMatrix_, distCoeffs_, rvec_pnp, tvec_pnp);

          expected_pattern_points.clear();

          projectPoints(worldPattern_, rvec_pnp, tvec_pnp, cameraMatrix_,
                        distCoeffs_, expected_pattern_points);
          found_pose = check_point(expected_pattern_points, found_blobs, 5);
        }
      }
    }
  }

  if (found_pose) {

    return validatePose(tvec_pnp, rvec_pnp);
  }
  return false;
}

//***********************************************************************************************
// Calculate Pose with prediction of new points
//***********************************************************************************************
bool ImageProcesser::trackedMatcher(std::vector<cv::Point2d> found_blobs) {

  bool found_pose = false;
  std::vector<cv::Point2d> projected_world_points;

  bool use_estimator;
  nh_private_.getParam("/ibis/image_processer/estimator_status", use_estimator);

  if (use_estimator) {

    projectPoints(worldPattern_, est_rot_, est_trans_, cameraMatrix_,
                  distCoeffs_, projected_world_points);
  } else {

    projectPoints(worldPattern_, curr_rot_, curr_trans_, cameraMatrix_,
                  distCoeffs_, projected_world_points);
  }

  double dist;
  double min_dist = 1000;
  int selector;
  double threshold_for_points = 50.0;

  std::vector<cv::Point2f> matched_image_points;
  std::vector<cv::Point3f> matched_world_points;

  cv::Mat rvec_pnp(3, 1, CV_64FC1, 0.0f);
  cv::Mat tvec_pnp(3, 1, CV_64FC1, 0.0f);

  for (int i = 0; i < projected_world_points.size(); i++) {

    for (int j = 0; j < found_blobs.size(); j++) {

      dist = sqrt(pow(projected_world_points.at(i).x - found_blobs.at(j).x, 2) +
                  pow(projected_world_points.at(i).y - found_blobs.at(j).y, 2));

      if (dist < min_dist) {
        min_dist = dist;
        selector = j;
      }
    }

    if (min_dist < threshold_for_points) {
      matched_world_points.push_back(worldPattern_.at(i));
      matched_image_points.push_back(found_blobs.at(selector));
    }

    min_dist = 1000;
  }

  if (matched_image_points.size() > 3) {

    solvePnPRansac(matched_world_points, matched_image_points, cameraMatrix_,
                   distCoeffs_, rvec_pnp, tvec_pnp);
  }

  return validatePose(tvec_pnp, rvec_pnp);
}

//***********************************************************************************************
// Check for obvious outliers of the pose
//***********************************************************************************************
bool ImageProcesser::validatePose(cv::Mat tvec, cv::Mat rvec) {

  if (tvec.at<double>(0, 0) != 0 || tvec.at<double>(1, 0) != 0 ||
      tvec.at<double>(2, 0) != 0) {

    if (tvec.at<double>(0, 0) < 5 && tvec.at<double>(1, 0) < 5 &&
        tvec.at<double>(2, 0) < 5) {

      if (tvec.at<double>(2, 0) > 0) {

        curr_trans_ = tvec;
        curr_rot_ = rvec;

        return true;
      }
    }
  }

  ROS_WARN("no valid pose found");

  return false;
}

//***********************************************************************************************
// Publish tf for the raw pattern transform
//***********************************************************************************************
void ImageProcesser::publishTFTransform(const ros::TimerEvent &event) {

  if (!first_time_flag_) {
    tf_broadcaster_.sendTransform(transform_odom_pattern_);
  }
}

} // namespace tethered_drone