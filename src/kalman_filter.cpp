#include "kalman_filter.h"
#include <iostream>
#include <cmath>
#define _USE_MATH_DEFINES

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // Matrix update
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // State update
  x_ = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  if( (px == 0 ) && (py ==0 ))
  {
    std::cout << "Error: divider!!" << std::endl;
    return;
  }

  float rho = sqrt( px*px + py*py );
  float phi = atan2( py, px );
  float rho_d = ( px*vx + py*vy ) / rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << 	rho,
		phi,
		rho_d;


  VectorXd y = z - z_pred;
  y(1) = NormalizeAngle( y(1) );  

  // Matrix update
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
 
  // State update
  x_ = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}

float KalmanFilter::NormalizeAngle( const float rad )
{
  float norm = rad;
  if( norm > M_PI )
  {
    std::cout << "norm_anlge: " << norm << " -> ";
    while( norm > M_PI )
    {
      norm -= 2*M_PI;
    }
    std::cout << norm << std::endl;
  }
  else if( norm < - M_PI )
  {
    std::cout << "norm_angle: " << norm << " -> ";
    while( norm < -M_PI )
    {
      norm += 2*M_PI;
    }
    std::cout << norm << std::endl;
  } 

  return norm;

}
