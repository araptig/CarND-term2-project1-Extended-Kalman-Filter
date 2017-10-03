#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include <iostream>

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix NOT USED
  Eigen::MatrixXd F_;

  // process covariance matrix NOT USED
  Eigen::MatrixXd Q_;

  // measurement matrix (used for laser only)
  Eigen::MatrixXd H_;

  // measurement covariance matrix (used for laser)
  Eigen::MatrixXd R_;

  // identity
  Eigen::MatrixXd I_;

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  // first time initialization for x and P
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in);

  //first time initialization for H and R (Laser)
  void Init(Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z, Eigen::MatrixXd &Hj, Eigen::MatrixXd &R_in);

  // compute h(x)
  Eigen::VectorXd compute_h(void);


};

#endif /* KALMAN_FILTER_H_ */
