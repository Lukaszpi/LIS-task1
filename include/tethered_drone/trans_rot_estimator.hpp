#ifndef TETHERED_DRONE_TRANSESTIMATOR
#define TETHERED_DRONE_TRANSESTIMATOR

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

namespace tethered_drone {

enum class EstimatorStatus { OK, OUTLIER, RESET };

//*******************************************************************************************************
// Translation Estimator
//*******************************************************************************************************

// Class for storing the prameters of the estimator
class TranslationEstimatorParams 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TranslationEstimatorParams()
      : kp_(1), kv_(1),
        initial_position_estimation_(Eigen::Vector3d::Zero()),
        initial_velocity_esitmation_(Eigen::Vector3d::Zero()) {}

  double kp_;
  double kv_;
  Eigen::Vector3d initial_position_estimation_;
  Eigen::Vector3d initial_velocity_esitmation_;
};

// Class for storing the results
class TranslationEstimatorResults
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TranslationEstimatorResults()
      : position_measured_result(Eigen::Vector3d::Zero()),
        position_old_result(Eigen::Vector3d::Zero()),
        position_estimate_result(Eigen::Vector3d::Zero()),
        velocity_estimate_result(Eigen::Vector3d::Zero()),
        velocity_old_result(Eigen::Vector3d::Zero()),
        measurement_outlier_flag_(false) {}

  Eigen::Vector3d position_measured_result;
  Eigen::Vector3d position_old_result; //returned
  Eigen::Vector3d position_estimate_result; //returned
  Eigen::Vector3d velocity_estimate_result; //returned
  Eigen::Vector3d velocity_old_result; //returned
  //
  //
  


  bool measurement_outlier_flag_;
};

// The estimator itself
class TranslationEstimator 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  TranslationEstimator();
  // Updatefunction of the estimator with new input postion and velocity
  EstimatorStatus updateEstimate(const Eigen::Vector3d &position_measured,
                                 const double timestamp);
  
  void reset();
  void setParameters(const TranslationEstimatorParams &estimator_parameters);
  // retrun results of the estiamtion step
  TranslationEstimatorResults getResults() const { return estimator_results_; }
  // get estimated position
  Eigen::Vector3d getEstimatedTransform() const {
    return estimated_position_cam_;
  }

  bool detectMeasurementOutlierSubsequent(const Eigen::Vector3d &pos_measured);
  bool detectMeasurementOutlierMohalanobis(
    const Eigen::Vector3d &position_measured,
    const Eigen::Matrix<double, 6, 1> x_prior,
    const Eigen::Matrix<double, 6, 6>  P_prior);
  TranslationEstimatorParams estimator_parameters_;

private:
  // parmeter and results objects
  TranslationEstimatorResults estimator_results_;
  

  // estimation of the estimator
  Eigen::Vector3d estimated_position_cam_;
  Eigen::Vector3d estimated_velocity_cam_;

  // Lukasz Edit
  Eigen::Matrix<double, 6, 1> estimated_prior_;
  Eigen::Matrix<double, 6, 1> estimated_posteriori_;
  Eigen::Matrix<double, 6, 6> covariance_prior_;
  Eigen::Matrix<double, 6, 6> covariance_posteriori_;


  // last measurements and first measurement
  double last_timestamp_;
  bool first_measurement_flag_;
  Eigen::Vector3d position_measured_old_;
  int outlier_counter_;

  void updatePosterioriKalmanEstimate( //should work
    const Eigen::Matrix<double, 6, 1> x_priori,
    const Eigen::Matrix<double, 6, 6> P_priori,
    const Eigen::Matrix<double, 3, 1> &x_measured, //no idea why it has &
    Eigen::Matrix<double, 6, 1> *x_posteriori,
    Eigen::Matrix<double, 6, 6> *P_posteriori);

  void updatePrioriKalmanEstimate( //should work
      const Eigen::Matrix<double, 6, 1> x_posteriori,
      const Eigen::Matrix<double, 6, 6> P_posteriori,
      const double dt,
      Eigen::Matrix<double, 6, 1> *x_priori,
      Eigen::Matrix<double, 6, 6> *P_priori);
};

//*******************************************************************************************************
// Rotation Estimator
//*******************************************************************************************************

class RotationEstimatorParameters
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RotationEstimatorParameters()
      : dorientationEstimateInitalCovariance_(100.0),
        drateEstimateInitalCovariance_(100.0),
        dorientationProcessCovariance_(0.000002), drateProcessCovariance_(10.0),
        orientationMeasurementCoavriance_(0.001),
        initalOrientationEstimate_(Eigen::Quaterniond::Identity()),
        initialRateEstimate_(Eigen::Vector3d::Zero()),
        initialDOrientationEsitmate_(Eigen::Vector3d::Zero()),
        initalDRateEstimate_(Eigen::Vector3d::Zero()),
        outputMinimalQuaternion_(false),
        outlier_rejection_subsequent_maximum_count_(30){};

  double dorientationEstimateInitalCovariance_;
  double drateEstimateInitalCovariance_;
  double dorientationProcessCovariance_;
  double drateProcessCovariance_;
  double orientationMeasurementCoavriance_;
  Eigen::Quaterniond initalOrientationEstimate_;
  Eigen::Vector3d initialRateEstimate_;
  Eigen::Vector3d initialDOrientationEsitmate_;
  Eigen::Vector3d initalDRateEstimate_;
  bool outputMinimalQuaternion_;
  int outlier_rejection_subsequent_maximum_count_;
};

class RotationEstiamtorResults
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RotationEstiamtorResults()
      : curr_orientation_measured_(Eigen::Quaterniond::Identity()),
        orientation_old_(Eigen::Quaterniond::Identity()),
        curr_orientation_estimate_(Eigen::Quaterniond::Identity()),
        rate_old_(Eigen::Vector3d::Zero()),
        curr_rate_estimate_(Eigen::Vector3d::Zero()),
        measurement_flip_flag_(false), qCovarianceTrace_(0.0){};

  Eigen::Quaterniond curr_orientation_measured_;
  Eigen::Quaterniond orientation_old_;
  Eigen::Quaterniond curr_orientation_estimate_;
  Eigen::Vector3d rate_old_;
  Eigen::Vector3d curr_rate_estimate_;
  bool measurement_flip_flag_;
  double qCovarianceTrace_;
  bool measurement_outlier_flag_;
};

class RotationEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RotationEstimator();

  EstimatorStatus updateEstimate(const Eigen::Quaterniond &oriantation_measured,
                                 // Eigen::Vector3d &rates_measured,
                                 const double timestamp);

  void reset();
  void setParameters(RotationEstimatorParameters &estimator_parameters);
  bool checkForCrash();

  RotationEstiamtorResults getResults() { return estimator_results_; }

  Eigen::Quaterniond getEstimatedRotation() const {
    return estimated_rotation_;
  }

private:
  RotationEstimatorParameters estimator_parameters_;
  RotationEstiamtorResults estimator_results_;

  // estiamtion of the orientation
  Eigen::Quaterniond estimated_rotation_;
  Eigen::Vector3d estimated_rate_;
  // error of the estiamtion
  Eigen::Vector3d err_rotation_est_;
  Eigen::Vector3d err_rate_est_;
  // covariance and error of it as well as measurement
  Eigen::Matrix<double, 6, 6> covariance_est_;
  Eigen::Matrix<double, 6, 6> process_covariance_est_;
  Eigen::Matrix<double, 4, 4> measurment_covariance_est_;

  // Last measurements and related things
  double last_timestamp_;
  Eigen::Quaterniond orientation_measured_old_;
  bool first_measurement_flag_;
  int outlier_counter_;
  bool measurement_outlier_flag_;

  Eigen::Matrix3d skewM(const Eigen::Vector3d &vec);

  void updateEstimatePropagateGlobalEstimate(
      const Eigen::Matrix<double, 7, 1> &x_old, const double dt,
      Eigen::Matrix<double, 7, 1> *x_priori);

  void updateEstimatePropagateErrorEstimate(
      const Eigen::Matrix<double, 6, 1> &dx_old,
      const Eigen::Matrix<double, 7, 1> &x_old, const double dt,
      Eigen::Matrix<double, 6, 1> *dx_priori);

  void updateEstimatePropagateErrorCovariance(
      Eigen::Matrix<double, 6, 6> &cov_old, const double dt,
      const Eigen::Matrix<double, 7, 1> &x_old,
      Eigen::Matrix<double, 6, 6> *covariance_priori);

  void updateEstimateUpdateErrorEstimate(
      const Eigen::Quaterniond &orientation_measured,
      const Eigen::Matrix<double, 7, 1> &x_priori,
      const Eigen::Matrix<double, 6, 1> &dx_priori,
      const Eigen::Matrix<double, 6, 6> &covariance_priori,
      Eigen::Matrix<double, 6, 1> *dx_measurement,
      Eigen::Matrix<double, 6, 6> *covariance_measurement);

  void updateEstimateRecombineErrorGlobal(
      const Eigen::Matrix<double, 7, 1> x_priori,
      Eigen::Matrix<double, 7, 1> *x_measurement,
      Eigen::Matrix<double, 6, 1> *dx_measurement);

  double quatRotMagniutde(const Eigen::Quaterniond &rotation);

  bool detectMeasurementOutlierMahalanobis(
      const Eigen::Quaterniond &orientation_measured,
      const Eigen::Matrix<double, 6, 6> &covariance);

  bool detectMeasurementOutlierSubsequent(
      const Eigen::Quaterniond &orientation_measured);

  double quaternionRotationMagnitude(const Eigen::Quaterniond &rotation);

  void makeCovSym(Eigen::Matrix<double, 6, 6> *covariance);
};

} // namespace tethered_drone

#endif /* TETHERED_DRONE_TRANSESTIMATOR */
