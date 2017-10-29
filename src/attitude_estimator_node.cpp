#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <kindr/minimal/quat-transformation.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tethered_drone/trans_rot_estimator.hpp>

using namespace tethered_drone;
using std::cout;
using std::endl;

// Convenience typedef
typedef kindr::minimal::QuatTransformation Transformation;

struct TransformationStamped {
  ros::Time stamp;
  Transformation transformation;
};

// Default values for parameters
static const std::string kDefaultLocalFrameName = "body";
static const std::string kDefaultGlobalFrameName = "odom";
constexpr double kDefaultTFPublishTime = 0.1;

class DataListener {

public:
  DataListener(ros::NodeHandle nh, ros::NodeHandle nh_private,
               TranslationEstimator *estimator_trans,
               RotationEstimator *estimator_rot)
      : translationestimator(estimator_trans), rotationestimator(estimator_rot),
        local_frame_name_(kDefaultLocalFrameName),
        global_frame_name_(kDefaultGlobalFrameName),
        tf_publish_time_(kDefaultTFPublishTime) {

    nh_private.getParam("estimator_kp",
                        translationestimator->estimator_parameters_.kp_);

    nh_private.getParam("estimator_kv",
                        translationestimator->estimator_parameters_.kv_);

    relative_sub_ = nh.subscribe("/ibis/image_processer/relative_pose", 100,
                                 &DataListener::transformCallback, this);

    result_pub_ = nh_private.advertise<geometry_msgs::TransformStamped>(
        "pose_filtered", 10);

    nh_private.getParam("local_frame_name", local_frame_name_);
    nh_private.getParam("global_frame_name", global_frame_name_);
    nh_private.getParam("tf_publish_time", tf_publish_time_);

    tf_timer_ = nh.createTimer(ros::Duration(tf_publish_time_),
                               &DataListener::publishTFTransform, this);
  }

  void transformCallback(const geometry_msgs::TransformStampedPtr &message) {

    tf::vectorMsgToEigen(message->transform.translation, position_calculated_);

    tf::quaternionMsgToEigen(message->transform.rotation,
                             orientation_calculated_);

    ros::Time timestamp = message->header.stamp;
    double timestamp_double = timestamp.toSec();

    translationestimator->TranslationEstimator::updateEstimate(
        position_calculated_, timestamp_double);
    rotationestimator->RotationEstimator::updateEstimate(
        orientation_calculated_, timestamp_double);

    Eigen::Vector3d translation_estimated =
        translationestimator->TranslationEstimator::getEstimatedTransform();

    Eigen::Quaterniond orientation_estimated =

        rotationestimator->RotationEstimator::getEstimatedRotation();

    //...........................................................................
    // For publishing the geometry msgs
    geometry_msgs::TransformStamped estimated_transform_Msgs;
    estimated_transform_Msgs.header = message->header;
    estimated_transform_Msgs.child_frame_id = message->child_frame_id;

    tf::vectorEigenToMsg(translation_estimated,
                         estimated_transform_Msgs.transform.translation);

    tf::quaternionEigenToMsg(orientation_estimated,
                             estimated_transform_Msgs.transform.rotation);

    result_pub_.publish(estimated_transform_Msgs);

    //...........................................................................
    // For publishing the tf
    transform_C_P_.transformation =
        Transformation(orientation_estimated, translation_estimated);

    tf::Transform tf_transform;
    tf::transformKindrToTF(transform_C_P_.transformation, &tf_transform);

    listener_.lookupTransform(global_frame_name_, "cam_down", ros::Time(0),
                              transform_odom_camdown_);

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

    transform_odom_pattern_filtered_ =
        tf::StampedTransform(T_O_P_gravity_aligned_tf, transform_C_P_.stamp,
                             global_frame_name_, local_frame_name_);

    if (first_time_flag_ = true) {
      first_time_flag_ = false;
    }
  }

private:
  ros::Subscriber relative_sub_;
  ros::Publisher result_pub_;

  Eigen::Matrix3d transform_Cam_B_; //not used

  Eigen::Vector3d velocity_measured_Cam_; //not used
  Eigen::Vector3d rate_measured_Cam_; // not used
  Eigen::Vector3d position_calculated_;
  Eigen::Quaterniond orientation_calculated_; // not used

  // tf related variables
  bool first_time_flag_;
  std::string local_frame_name_;
  std::string global_frame_name_;
  double tf_publish_time_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  tf::TransformListener listener_;
  tf::StampedTransform transform_odom_camdown_;
  tf::StampedTransform transform_odom_pattern_filtered_;
  TransformationStamped transform_C_P_;

  std::unique_ptr<TranslationEstimator> translationestimator;
  std::unique_ptr<RotationEstimator> rotationestimator;

  // Publishes the resulting transform
  void publishTFTransform(const ros::TimerEvent &event) {

    if (!first_time_flag_) {
      tf_broadcaster_.sendTransform(transform_odom_pattern_filtered_);
    }
  }
};

// Standard C++ entry point
int main(int argc, char **argv) {

  // Announce this program to the ROS master
  ros::init(argc, argv, "estimator");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  TranslationEstimator estimator_trans;

  RotationEstimator estimator_rot;

  DataListener dataListener(nh, private_nh, &estimator_trans, &estimator_rot);

  ros::spin();

  return 0;
}
