#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}
Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check estimations not empty
	if (estimations.size() == 0)
	{
	    cout << "estimation vector empty" << endl;
	    return(rmse);
	}

	//  check estimation  size equals ground truth size
	if (estimations.size() != ground_truth.size())
	{
	    cout << "estimation vector size does not match ground_truth" << endl;
	    return(rmse);
	}

    VectorXd error(4);
	for(int i=0; i < estimations.size(); ++i)
	{
	    error = estimations[i]-ground_truth[i];
	    error = error.array()*error.array();
        rmse = rmse + error;
	}
	rmse = rmse / float(estimations.size());
	rmse = rmse.array().sqrt();
	//cout << "mse = " << rmse << endl;
	return(rmse);
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// distance squared
	float pos_2 = px*px + py*py;

	MatrixXd Hj(3,4);
	// initialize Hj
	Hj << 0, 0, 0, 0,
	      0, 0, 0, 0,
	      0, 0, 0, 0;
	//check division by zero
	if (pos_2 > 0.0001)
	{//generate Hj
		// temporary values
	    float pos_s = sqrt(pos_2);
	    float pos_3 = pos_s*pos_2;
	    // set Hj
	    Hj(0,0) = px/pos_s;
	    Hj(0,1) = py/pos_s;
	    Hj(1,0) = -py/pos_2;
	    Hj(1,1) = px/pos_2;
	    Hj(2,2) = px/pos_s;
	    Hj(2,3) = py/pos_s;
	    Hj(2,0) = (py*(vx*py-vy*px))/pos_3;
	    Hj(2,1) = (px*(vy*px-vx*py))/pos_3;
	}//generate Hj
	return(Hj);
}
