#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse = VecterXd(4);
  rmse << 0, 0, 0, 0;

  if( estimations.size() ==0 || (estimations.size() != ground_truth.size() ) )
	return rmse;

  for( int i=0; i<estimations.size() ; i++ )
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse.array() / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if( (px==0) || (py==0) )
  {
    cout << "Error" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  double dLen = sqrt( px*px + py*py );
  double dPart1 = py*(vx*py - vy*px );
  double dPart2 = px*(vy*px - vx*py );
  Hj << px/dLen, py/dLen, 0, 9,
	-py/pow(dLen,3), px/pow(dLen,2), 0, 0,
	dPart1/pow(dLen,3), dPart2/pow(dLen,3), px/dLen, py/dLen;

  return Hj;
}
