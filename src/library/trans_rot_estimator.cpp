#include "tethered_drone/trans_rot_estimator.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <glog/logging.h>
#include <ros/console.h>

using std::cout;
using std::endl;

namespace tethered_drone {

//*******************************************************************************************************
// Translation Estimator
//*******************************************************************************************************

TranslationEstimator::TranslationEstimator()
    : estimator_parameters_(), estimator_results_()
{

  TranslationEstimator::reset();
}

EstimatorStatus
TranslationEstimator::updateEstimate(const Eigen::Vector3d &transform_measured,
                                     const double timestamp) 
{

  if (first_measurement_flag_) 
  {
    first_measurement_flag_ = false;
    last_timestamp_ = timestamp;
    estimated_position_cam_ = transform_measured;
    estimated_prior_ << transform_measured, 0, 0, 0;
    estimated_posteriori_ << transform_measured, 0, 0, 0;
    covariance_prior_ = Eigen::Matrix<double, 6, 6>::Identity();
    covariance_posteriori_ = Eigen::Matrix<double, 6, 6>::Identity();

    return EstimatorStatus::OK;
  }

  // Rewriting old estimate
  estimator_results_.position_old_result = estimator_results_.position_estimate_result;
  estimator_results_.velocity_old_result = estimator_results_.velocity_estimate_result;

  // calculation of time passed since last measurement
  double dt = timestamp - last_timestamp_;
  last_timestamp_ = timestamp;

  // propagate error state estimation
  Eigen::Matrix<double, 6, 1> x_m;
  Eigen::Matrix<double, 6, 6> P_m;
  Eigen::Matrix<double, 6, 1> x_p;
  Eigen::Matrix<double, 6, 6> P_p;
  x_m = estimated_posteriori_;
  P_m = covariance_posteriori_;


  updatePrioriKalmanEstimate(x_m, P_m, dt, &x_p, &P_p);

  // Detecting outlier measurements
  bool measurement_update_flag  = 1; //
  bool future_measurement_update_flag =  detectMeasurementOutlierMohalanobis(transform_measured, x_p, P_p);

  // if no outlier detected, do update
  if (future_measurement_update_flag)
  {
    updatePosterioriKalmanEstimate(x_p,
      P_p, transform_measured, &x_m, &P_m);

    // save private variables
    estimated_prior_ = x_p;
    estimated_posteriori_ = x_m;
    covariance_prior_ = P_p;
    covariance_posteriori_ = P_m;

    estimated_position_cam_ = x_m.block<3, 1>(0, 0);
    estimated_velocity_cam_ = x_m.block<3, 1>(3, 0);

    // saving the results
    estimator_results_.position_estimate_result = estimated_position_cam_;
    estimator_results_.velocity_estimate_result = estimated_velocity_cam_;

    return EstimatorStatus::OK; 
  }
  else
  {

    estimated_position_cam_ = x_p.block<3, 1>(0, 0);
    estimated_velocity_cam_ = x_p.block<3, 1>(3, 0);

    // saving the results
    estimator_results_.position_estimate_result = estimated_position_cam_;
    estimator_results_.velocity_estimate_result = estimated_velocity_cam_;

    return EstimatorStatus::OUTLIER; // was initially outlier then it didn't save the reading when still was quite good
  }
}

void TranslationEstimator::updatePrioriKalmanEstimate( //should work
    const Eigen::Matrix<double, 6, 1> x_posteriori,
    const Eigen::Matrix<double, 6, 6> P_posteriori,
    const double dt,
    Eigen::Matrix<double, 6, 1> *x_priori,
    Eigen::Matrix<double, 6, 6> *P_priori) {

  // extracting stuff
  Eigen::Matrix<double, 6, 1> x_m;
  Eigen::Matrix<double, 6, 6> P_m;;
  x_m = x_posteriori;
  P_m = P_posteriori;

  Eigen::Matrix<double, 6, 1> x_p;
  Eigen::Matrix<double, 6, 6> P_p;
  
  // constructing the state matrix A
  Eigen::Matrix<double, 6, 6> A;
  A.setZero();
  A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();

  // Constructing the Process Noise Matrix Q
  Eigen::Matrix<double, 6, 6> Q;
  double o_x = 0.019;
  double o_z = 0.029;
  double o_v = 0.001;

  Q << o_x, 0, 0, 0, 0, 0,
        0, o_x, 0, 0, 0, 0,
        0, 0, o_z, 0, 0, 0,
        0, 0, 0, o_v, 0, 0,
        0, 0, 0, 0, o_v, 0,
        0, 0, 0, 0, 0, o_v;

  //[o_x o_x o_z o_v o_v o_v]
  Q = dt * Q;
  
  // priori update
  x_p = A*x_m;
  P_p = A*P_m*A.transpose() + Q;

  // writing to "output"
  *x_priori = x_p;
  *P_priori = P_p;
}

void TranslationEstimator::updatePosterioriKalmanEstimate( //should work
    const Eigen::Matrix<double, 6, 1> x_priori,
    const Eigen::Matrix<double, 6, 6> P_priori,
    const Eigen::Matrix<double, 3, 1> &x_measured, //no idea why it has &
    Eigen::Matrix<double, 6, 1> *x_posteriori,
    Eigen::Matrix<double, 6, 6> *P_posteriori) {
  // extracting stuff
  Eigen::Matrix<double, 6, 1> x_p = x_priori;
  Eigen::Matrix<double, 6, 6> P_p = P_priori;
  Eigen::Matrix<double, 3, 1> z = x_measured;

  Eigen::Matrix<double, 6, 1> x_m;
  Eigen::Matrix<double, 6, 6> P_m;

  // Construct the measurement matrix H
  Eigen::Matrix<double, 3, 6> H;
  H.setZero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // Constructing the Reading Noise Matrix R
  Eigen::Matrix<double, 3, 3> R;
  double r_x = 0.0017;
  double r_z = 1.5;
  // Getting values for R amtrix
  double radius = sqrt(sqrt(pow(x_p(0,0),2)+pow(x_p(1,0),2)));
  R.setZero();
  R(0,0) = r_x;
  R(1,1) = r_x;
  R(2,2) = r_z*radius;
  
  // Constructing the Kalman Gain Matrix K
  Eigen::Matrix<double, 6, 3> K;
  K = P_p * H.transpose() * (H * P_p * H.transpose() + R).inverse();

  // posteriori update
  x_m = x_p + K*(z - H*x_p);
  P_m = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P_p;

  // writing to "output"
  *x_posteriori = x_m;
  *P_posteriori = P_m;
}

void TranslationEstimator::setParameters(
    const TranslationEstimatorParams &estimator_parameters)
{

  estimated_position_cam_ = estimator_parameters_.initial_position_estimation_;
  estimator_parameters_ = estimator_parameters;
}

void TranslationEstimator::reset() //should work
{

  last_timestamp_ = -1.0;
  first_measurement_flag_ = true;
  estimated_prior_.setZero();
  estimated_posteriori_.setZero();
  covariance_prior_ = Eigen::Matrix<double, 6, 6>::Identity();
  covariance_posteriori_ = Eigen::Matrix<double, 6, 6>::Identity();
}

bool TranslationEstimator::detectMeasurementOutlierMohalanobis(
    const Eigen::Vector3d &position_measured,
    const Eigen::Matrix<double, 6, 1> x_prior,
    const Eigen::Matrix<double, 6, 6> P_prior)
{
  if (first_measurement_flag_)
  {
    first_measurement_flag_ = false;
    position_measured_old_ = position_measured;
    return false;
  }
  Eigen::Vector3d mohalanobis_mean = x_prior.block<3,1>(0, 0);
  Eigen::Matrix3d mohalanobis_cov = P_prior.block<3,3>(0, 0);
  Eigen::Vector3d observation_distance = position_measured - mohalanobis_mean;

  double mohalanobis_distance =  sqrt(observation_distance.transpose() * mohalanobis_cov * observation_distance);
  if(mohalanobis_distance > 0.7)
    return 0;
  else
    return 1;
}




bool TranslationEstimator::detectMeasurementOutlierSubsequent(
    const Eigen::Vector3d &position_measured) {

  if (first_measurement_flag_)
  {
    first_measurement_flag_ = false;
    position_measured_old_ = position_measured;
    return false;
  }

  Eigen::Vector3d position_error = position_measured_old_ - position_measured;

  bool measurement_outlier_flag = (position_error.norm() >= 0.7);

  //double position_error = x_measured(2) - x_priori(2);
  //if(position_error<0)//absolute hack
  //{
  //  position_error = -position_error;
  //}
  //
  //bool measurement_outlier_flag = (position_error >= 100 * P_priori(2,2));
  // return measurement_outlier_flag

  if (outlier_counter_ > 20)
  {
    measurement_outlier_flag = false;
  }

  estimator_results_.measurement_outlier_flag_ = measurement_outlier_flag;

  if (measurement_outlier_flag)
  {
    ++outlier_counter_;
    return true;
  }
  else
  {
    position_measured_old_ = position_measured;
    outlier_counter_ = 0;
    return false;
  }
}

//*******************************************************************************************************
// Rotation Estimator
//*******************************************************************************************************

RotationEstimator::RotationEstimator()
    : estimated_rotation_(), err_rotation_est_(){};

EstimatorStatus RotationEstimator::updateEstimate(
    const Eigen::Quaterniond &orientation_measured, const double timestamp)
{

  // write measurement to result object
  estimator_results_.curr_orientation_measured_ = orientation_measured;
  // writing old estimates to result object
  estimator_results_.orientation_old_ = estimated_rotation_;
  estimator_results_.rate_old_ = estimated_rate_;

  // initialize the estimator if first measurement flag
  if (first_measurement_flag_)
  {
    first_measurement_flag_ = false;
    orientation_measured_old_ = orientation_measured;
    last_timestamp_ = timestamp;
    estimated_rotation_ = orientation_measured;
    return EstimatorStatus::OK;
  }

  // calculate times
  double dt = timestamp - last_timestamp_;
  last_timestamp_ = timestamp;

  // propagate state estimation
  Eigen::Matrix<double, 7, 1> x_old;
  Eigen::Matrix<double, 7, 1> x_p;
  x_old << estimated_rotation_.coeffs(), estimated_rate_;

  updateEstimatePropagateGlobalEstimate(x_old, dt, &x_p);

  // propagate error state estimation
  Eigen::Matrix<double, 6, 1> dx_old;
  Eigen::Matrix<double, 6, 1> dx_p;
  dx_old << err_rotation_est_, err_rate_est_;

  updateEstimatePropagateErrorEstimate(dx_old, x_old, dt, &dx_p);

  // propagate estimate covariance
  Eigen::Matrix<double, 6, 6> P_old;
  Eigen::Matrix<double, 6, 6> P_p;
  P_old = covariance_est_;
  updateEstimatePropagateErrorCovariance(P_old, dt, x_old, &P_p);

  // define posteriori variables
  Eigen::Matrix<double, 7, 1> x_m;
  Eigen::Matrix<double, 6, 1> dx_m;
  Eigen::Matrix<double, 6, 6> P_m;

  // detect if outlier is present
  bool measurement_update_flag;
  measurement_update_flag =
      !detectMeasurementOutlierSubsequent(orientation_measured); // should it be Mahalanobis

  if (measurement_update_flag)
  {
    // calculate global update
    updateEstimateUpdateErrorEstimate(orientation_measured, x_p, dx_p, P_p,
                                      &dx_m, &P_m);
    updateEstimateRecombineErrorGlobal(x_p, &x_m, &dx_m);
  }
  else
  {

    updateEstimateRecombineErrorGlobal(x_p, &x_m, &dx_p);

    P_m = P_p;
  }

  // estimated variables from posteriori state
  estimated_rotation_ = Eigen::Quaterniond(x_m.block<4, 1>(0, 0));
  estimated_rate_ = x_m.block<3, 1>(4, 0);
  covariance_est_ = P_m;

  // writing to intermidiate result structure
  estimator_results_.curr_orientation_estimate_ = estimated_rotation_;
  estimator_results_.curr_rate_estimate_ = estimated_rate_;

  bool crashFlag = checkForCrash();

  if (crashFlag)
  {
    RotationEstimator::reset();
    return EstimatorStatus::RESET;
  }
  else
  {

    return EstimatorStatus::OK;
  }
}

//******************** end of void updateEstimate()

void RotationEstimator::updateEstimatePropagateGlobalEstimate(
    const Eigen::Matrix<double, 7, 1> &x_old, const double dt,
    Eigen::Matrix<double, 7, 1> *x_priori) {

  CHECK_NOTNULL(x_priori);
  // extracting stuff
  Eigen::Quaterniond orientation_estimate_old =
      Eigen::Quaterniond(x_old.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate_old = x_old.block<3, 1>(4, 0);
  // Converting roll rate to quaternion
  Eigen::Quaterniond rate_estimate_quat = Eigen::Quaterniond(
      0, rate_estimate_old.x(), rate_estimate_old.y(), rate_estimate_old.z());
  // roll rate and orientation propagation
  Eigen::Quaterniond orientation_estimate_priori = Eigen::Quaterniond(
      orientation_estimate_old.coeffs() +
      (0.5 * (orientation_estimate_old * rate_estimate_quat).coeffs()) *
          dt); //*dt
  Eigen::Vector3d rate_estimate_priori =
      rate_estimate_old; // might have to change because of knowledge of rates

  // normalizing the quaternion
  orientation_estimate_priori.normalize();

  // writing to output
  *x_priori << orientation_estimate_priori.coeffs(), rate_estimate_priori;
}

void RotationEstimator::updateEstimatePropagateErrorEstimate(
    const Eigen::Matrix<double, 6, 1> &dx_old,
    const Eigen::Matrix<double, 7, 1> &x_old, const double dt,
    Eigen::Matrix<double, 6, 1> *dx_priori) {

  // check input
  CHECK_NOTNULL(dx_priori);

  // extracting stuff
  Eigen::Quaterniond orientation_estimate =
      Eigen::Quaterniond(x_old.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate = x_old.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dorientation_estimate = dx_old.block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> drate_estimate = dx_old.block<3, 1>(3, 0);

  // propagate error estiamtion
  Eigen::Vector3d dorientation_estimate_priori_roc =
      -rate_estimate.cross(dorientation_estimate) + 0.5 * drate_estimate;

  Eigen::Vector3d drate_estimate_priori_roc = Eigen::Vector3d::Zero();

  Eigen::Vector3d dorientation_estimate_priori =
      dorientation_estimate + dorientation_estimate_priori_roc * dt; //*dt
  Eigen::Vector3d drate_Estimate_priori =
      drate_estimate + drate_estimate_priori_roc * dt; //*dt

  // writing to output
  *dx_priori << dorientation_estimate_priori, drate_Estimate_priori;
}

void RotationEstimator::updateEstimatePropagateErrorCovariance(
    Eigen::Matrix<double, 6, 6> &cov_old, const double dt,
    const Eigen::Matrix<double, 7, 1> &x_old,
    Eigen::Matrix<double, 6, 6> *covariance_priori) {

  // check input
  CHECK_NOTNULL(covariance_priori);

  // extracting stuff
  Eigen::Quaterniond orientation_estimate =
      Eigen::Quaterniond(x_old.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate = x_old.block<3, 1>(4, 0);

  // constructing system matrixes
  Eigen::Matrix<double, 6, 6> A;
  Eigen::Matrix<double, 6, 6> L;

  A << -1 * skewM(rate_estimate), 0.5 * Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero();
  L << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

  // propagation of covariance
  Eigen::Matrix<double, 6, 6> covariance_priori_propagation;
  Eigen::Matrix<double, 6, 6> covariance_priori_addition;
  Eigen::Matrix<double, 6, 6> A_d =
      Eigen::Matrix<double, 6, 6>::Identity() + A * dt; //*dt

  covariance_priori_propagation = A_d * cov_old + A_d.transpose();
  covariance_priori_addition =
      L * process_covariance_est_ * (L.transpose()) * dt; //*dt

  *covariance_priori =
      covariance_priori_addition + covariance_priori_propagation;

  makeCovSym(covariance_priori);
}

void RotationEstimator::updateEstimateUpdateErrorEstimate(
    const Eigen::Quaterniond &orientation_measured,
    const Eigen::Matrix<double, 7, 1> &x_priori,
    const Eigen::Matrix<double, 6, 1> &dx_priori,
    const Eigen::Matrix<double, 6, 6> &covariance_priori,
    Eigen::Matrix<double, 6, 1> *dx_measurement,
    Eigen::Matrix<double, 6, 6> *covariance_measurement) {

  // Extracting stuff
  Eigen::Quaterniond orientation_estimate_priori =
      Eigen::Quaterniond(x_priori.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate_priori = x_priori.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dorientation_estimate_priori =
      dx_priori.block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> drate_estimate_priori =
      dx_priori.block<3, 1>(3, 0);

  // constructing system matrixes
  Eigen::Matrix<double, 4, 3> Hdq;
  Eigen::Matrix<double, 4, 6> H;
  Eigen::Vector3d orientation_estimate_priori_vector =
      orientation_estimate_priori.vec();
  Hdq << orientation_estimate_priori.w() *
                 Eigen::Matrix<double, 3, 3>::Identity() +
             skewM(orientation_estimate_priori_vector),
      -orientation_estimate_priori.vec().transpose(); //Error this should be before skew
  H << Hdq, Eigen::Matrix<double, 4, 3>::Zero();

  // calculate the measured error quat
  Eigen::Quaterniond error_orientation =
      orientation_measured * orientation_estimate_priori.inverse();
  // look for sign of quat and label flag depending
  Eigen::Quaterniond orientation_predicted;
  if (error_orientation.w() >= 0) {
    estimator_results_.measurement_flip_flag_ = false;
    orientation_predicted =
        Eigen::Quaterniond(Hdq * dorientation_estimate_priori +
                           orientation_estimate_priori.coeffs());
  } else {
    estimator_results_.measurement_flip_flag_ = true;
    orientation_predicted =
        Eigen::Quaterniond(Hdq * dorientation_estimate_priori +
                           orientation_estimate_priori.coeffs());
  }

  // measurement residual
  Eigen::Vector4d measurement_residual;
  measurement_residual =
      orientation_measured.coeffs() - orientation_predicted.coeffs();

  // Kalman Gain
  Eigen::Matrix<double, 4, 4> S =
      H * covariance_priori * H.transpose() + measurment_covariance_est_;
  Eigen::Matrix<double, 4, 6> b = H * covariance_priori;
  Eigen::Matrix<double, 4, 4> A = S.transpose();
  Eigen::Matrix<double, 4, 6> x = A.colPivHouseholderQr().solve(b);
  Eigen::Matrix<double, 6, 4> K = x.transpose();

  // correcting the state
  *dx_measurement = dx_priori + K * measurement_residual;

  // updating the covariance
  *covariance_measurement =
      (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * covariance_priori;
  makeCovSym(covariance_measurement);
}

void RotationEstimator::updateEstimateRecombineErrorGlobal(
    const Eigen::Matrix<double, 7, 1> x_priori,
    Eigen::Matrix<double, 7, 1> *x_measurement,
    Eigen::Matrix<double, 6, 1> *dx_measurement) {

  // extracting stuff
  Eigen::Quaterniond orientation_estimate_priori =
      Eigen::Quaterniond(x_priori.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate_priori = x_priori.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dorientation_estimate_measurement =
      dx_measurement->block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> drate_estimate_measurement =
      dx_measurement->block<3, 1>(0, 0); // shouldn't be (3,0)?

  // error Quaternion
  Eigen::Quaterniond dorientation_estimate_measurement_quaternion;
  if (estimator_results_.measurement_flip_flag_ == false) {
    dorientation_estimate_measurement_quaternion =
        Eigen::Quaterniond(1.0, dorientation_estimate_measurement.x(),
                           dorientation_estimate_measurement.y(),
                           dorientation_estimate_measurement.z());
  } else {
    dorientation_estimate_measurement_quaternion =
        Eigen::Quaterniond(-1.0, dorientation_estimate_measurement.x(),
                           dorientation_estimate_measurement.y(),
                           dorientation_estimate_measurement.z());
  }

  // estimated error states to correct the global states
  Eigen::Quaterniond orientation_estimate_measurement =
      orientation_estimate_priori *
      dorientation_estimate_measurement_quaternion;
  Eigen::Matrix<double, 3, 1> rate_estimate_measurement =
      rate_estimate_priori + drate_estimate_measurement;

  // normalize the quternion
  orientation_estimate_measurement.normalize();

  // writing to "output"
  *x_measurement << orientation_estimate_measurement.coeffs(),
      rate_estimate_measurement;
  *dx_measurement = Eigen::Matrix<double, 6, 1>::Zero();
}

void RotationEstimator::makeCovSym(Eigen::Matrix<double, 6, 6> *covariance) {

  CHECK_NOTNULL(covariance);

  *covariance = (*covariance + covariance->transpose()) / 2;
}

Eigen::Matrix3d RotationEstimator::skewM(const Eigen::Vector3d &vec) {

  Eigen::Matrix3d vector_cross;

  vector_cross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return vector_cross;
}

bool RotationEstimator::checkForCrash() {
  double nan_test = covariance_est_.sum();
  return std::isnan(nan_test);
}

void RotationEstimator::reset() {
  // reset old measurements
  estimated_rotation_ = estimator_parameters_.initalOrientationEstimate_;
  estimated_rate_ = estimator_parameters_.initialRateEstimate_;
  err_rotation_est_ = estimator_parameters_.initialRateEstimate_;
  err_rate_est_ = estimator_parameters_.initalDRateEstimate_;

  // reset covariance
  covariance_est_
      << estimator_parameters_.dorientationEstimateInitalCovariance_ *
             Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      estimator_parameters_.drateEstimateInitalCovariance_ *
          Eigen::Matrix3d::Identity();

  process_covariance_est_
      << estimator_parameters_.dorientationProcessCovariance_ *
             Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      estimator_parameters_.drateProcessCovariance_ *
          Eigen::Matrix3d::Identity();

  measurment_covariance_est_
      << estimator_parameters_.orientationMeasurementCoavriance_ *
             Eigen::Matrix4d::Identity();

  // reset rest of parameters
  last_timestamp_ = -1.0;
  orientation_measured_old_ = estimator_parameters_.initalOrientationEstimate_;
  first_measurement_flag_ = true;
  outlier_counter_ = 0;
}


bool RotationEstimator::detectMeasurementOutlierMahalanobis(
    const Eigen::Quaterniond &orientation_measured,
    const Eigen::Matrix<double, 6, 6> &covariance) {
  // Initializing the outlier flag
  bool measurement_outlier_flag = false;
  // Calculating the mahalanobis distance (3x3 matrix - Should be fine for
  // direct inverse).
  Eigen::Quaterniond q_Z_B =
      orientation_measured * estimated_rotation_.inverse();
  Eigen::Vector3d dq_Z_B_ = q_Z_B.vec();
  Eigen::Matrix<double, 3, 3> dq_covariance = covariance.block<3, 3>(0, 0);
  double q_Z_B_mahalanobis_distance =
      sqrt(dq_Z_B_.transpose() * dq_covariance.inverse() * dq_Z_B_);
  // Detecting outlier
  measurement_outlier_flag = q_Z_B_mahalanobis_distance >= 5.0;
  // Writing the intermediate results (for debug)
  // estimator_results_.q_Z_Z1_magnitude_ = -1.0;
  // estimator_results_.q_Z_B_mahalanobis_distance_ =
  // q_Z_B_mahalanobis_distance;
  // Saving the flag to the intermediate results structure
  estimator_results_.measurement_outlier_flag_ = measurement_outlier_flag;
  // If rotation too great indicate that measurement is corrupted
  if (measurement_outlier_flag) {
    ++outlier_counter_;
    return true;
  } else {
    // If measurement valid. Overwriting the old measurement.
    orientation_measured_old_ = orientation_measured;
    outlier_counter_ = 0;
    return false;
  }
}

bool RotationEstimator::detectMeasurementOutlierSubsequent(
    const Eigen::Quaterniond &orientation_measured) {
  // Initializing the outlier flag
  bool measurement_outlier_flag = false;
  // Constructing the quaternion representing the rotation between subsquent
  // measurements
  Eigen::Quaterniond q_Z_Z1 =
      orientation_measured * orientation_measured_old_.inverse();
  // Calculating the quaternion magnitude
  double q_Z_Z1_magnitude = quaternionRotationMagnitude(q_Z_Z1);
  // Detecting outlier
  measurement_outlier_flag = q_Z_Z1_magnitude >= 30.0 * M_PI / 180.0;
  // After a certain number of measurements have been ignored in a row
  // we assume we've made a mistake and accept the measurement as valid.
  if (outlier_counter_ >=
      estimator_parameters_.outlier_rejection_subsequent_maximum_count_) {
    measurement_outlier_flag = false;
  }
  // Writing the intermediate results (for debug)
  // estimator_results_.q_Z_Z1_magnitude_ = q_Z_Z1_magnitude;
  // estimator_results_.q_Z_B_mahalanobis_distance_ = -1.0;
  // If rotation too great indicate that measurement is corrupted
  if (measurement_outlier_flag) {
    ++outlier_counter_;
    return true;
  } else {
    // If measurement valid. Overwriting the old measurement.
    orientation_measured_old_ = orientation_measured;
    outlier_counter_ = 0;
    return false;
  }
}

double RotationEstimator::quaternionRotationMagnitude(
    const Eigen::Quaterniond &rotation) {
  // Extracting the quaternion magnitude component
  double positive_rotation_return = 2 * acos(rotation.w());
  double negative_rotation_return = 2 * M_PI - 2 * acos(rotation.w());
  if (positive_rotation_return <= M_PI) {
    return positive_rotation_return;
  } else {
    return negative_rotation_return;
  }
}

} // namespace tethered_drone