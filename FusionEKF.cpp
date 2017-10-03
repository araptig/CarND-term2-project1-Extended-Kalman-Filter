#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF()
{//constructor

  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);           // init later
  H_laser_ = MatrixXd(2, 4);
  F_       = MatrixXd::Identity(4,4); // barebone F
  Qa_      = MatrixXd(2, 2);          // process noise

  {//laser R & H
	  R_laser_ << 0.0225, 0,
                  0, 0.0225;
	  H_laser_ << 1, 0, 0, 0,
 		          0, 1, 0, 0;
	  ekf_.Init(H_laser_,R_laser_);
  }

  {//measurement covariance matrix - radar
	  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  }


  {// deterministic part of process noise
	  Qa_ << 9, 0,
		0, 9;
  }
}//constructor

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{//ProcessMeasurement

  // initialization
  if (!is_initialized_)
  {//initialize
	initialize(measurement_pack);
    return;
  }//initialize

  {// prediction
	  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

	  {// set F
		  F_(0,2) = dt;
		  F_(1,3) = dt;
	  }

	  {// set Q
		  MatrixXd G(4,2);
		  G << dt*dt/2.0, 0,
			   0, dt*dt/2.0,
			   dt, 0,
		       0 , dt;
		  Q_ = G * Qa_ * G.transpose();
	  }

	  // call EKF
	  ekf_.Predict(F_,Q_);

	  previous_timestamp_ = measurement_pack.timestamp_;
  }//predict


  {//update
	  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	  {//radar, use EKF
		  cout << endl << "radar" << endl;
		  Hj_ = tools.CalculateJacobian(ekf_.x_);
		  ekf_.UpdateEKF(measurement_pack.raw_measurements_,Hj_,R_radar_);
	  }//radar
	  else
	  {// laser, use KF
		  cout << endl << "laser" << endl;
		  ekf_.Update(measurement_pack.raw_measurements_);
	  }// laser
  }//update

  {// print the output
	  cout << "x_ = " << ekf_.x_ << endl;
	  cout << "P_ = " << ekf_.P_ << endl;
  }
}

void FusionEKF::initialize(const MeasurementPackage &measurement_pack)
{//initialize
	VectorXd xi(4);     // initial state
	MatrixXd Pi(4,4);   // initial covariance
	xi << 0, 0, 0, 0;
    Pi << 0, 0, 0, 0,
    	  0, 0, 0, 0,
		  0, 0, 0, 0,
		  0, 0, 0, 0;

	if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
	{//laser
		xi(0) = measurement_pack.raw_measurements_(0);
		xi(1) = measurement_pack.raw_measurements_(1);
	}//laser
	else
	{//radar
		float rho     = measurement_pack.raw_measurements_(0);
		float phi     = measurement_pack.raw_measurements_(1);
		float rho_dot = measurement_pack.raw_measurements_(2);

		float px      = rho * cos(phi);
		float py      = -rho * sin(phi);
		xi(0)         = px;
		xi(1)         = py;
	}//radar

	{// covariance: position information but no velocity information
		Pi(0,0) = 1.0;
		Pi(1,1) = 1.0;
		Pi(2,2) = 1000.0;
		Pi(3,3) = 1000.0;
	}

	ekf_.Init(xi,Pi);             // set xi, PI into ekf_
	is_initialized_ = true;
	previous_timestamp_ = measurement_pack.timestamp_;
	cout << "EKF initialized: " << endl;
} // initialize



