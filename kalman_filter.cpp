#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter()
{
	I_ = MatrixXd::Identity(4,4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{// not used
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in)
{
  x_ = x_in;
  P_ = P_in;
}

void KalmanFilter::Init(MatrixXd &H_in, MatrixXd &R_in)
{
  H_ = H_in;
  R_ = R_in;
}

void KalmanFilter::Predict(MatrixXd &F_in, MatrixXd &Q_in)
{//predict
	x_ = F_in * x_;                           // k|k --> k+1|k
	P_ = F_in * P_ * F_in.transpose() + Q_in; // k|k --> k+1|k
}//predict

void KalmanFilter::Update(const VectorXd &z)
{//KF
	VectorXd y = z - H_ * x_;                  // residual
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();

	x_ = x_ + K * y;
	P_ = (I_ - K*H_)*P_;
}//KF

void KalmanFilter::UpdateEKF(const VectorXd &z, MatrixXd &Hj, MatrixXd &R_in)
{//radar
	VectorXd y = z - compute_h();       // residual

	// -pi <= y(1) <= pi
	while (y(1) > M_PI)
	{
		y(1) = y(1) - M_PI;
	}
	while (y(1) < -M_PI)
	{
		y(1) = y(1) + M_PI;
	}
	MatrixXd S = Hj * P_ * Hj.transpose() + R_in;
	MatrixXd K = P_ * Hj.transpose() * S.inverse();

	x_ = x_ + K * y;
	P_ = (I_ - K*Hj)*P_;
}//radar


VectorXd KalmanFilter::compute_h(void)
{// predicted observations
	double temp = sqrt(double(x_(0)*x_(0)+x_(1)*x_(1)));
	VectorXd h(3);
	h(0) = temp;
	if (temp > 0.001)
	{
		h(1) = atan2(double(x_(1)),double(x_(0)));
		h(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / temp;
	}
	else
	{
		h(1) = 0;
		h(2) = 0;
	}
	return(h);
}// predicted observations

