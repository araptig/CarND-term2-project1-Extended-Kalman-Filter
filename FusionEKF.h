#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF
{
public:
  FusionEKF();
  virtual ~FusionEKF();

  // Run the whole flow of the EKF from here.
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // Kalman Filter update and prediction math lives in here.
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  // previous timestamp
  long long previous_timestamp_;
  // tool includes Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  Eigen::MatrixXd F_;              // state matrix
  Eigen::MatrixXd Qa_;             // process covariance, deterministic
  Eigen::MatrixXd Q_;              // process noise covariance

  void initialize(const MeasurementPackage &measurement_pack);
};

#endif 
