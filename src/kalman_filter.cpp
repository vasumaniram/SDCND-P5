#include "kalman_filter.h"
#include <iostream>
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::atan2;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  const VectorXd z_pred = H_ * x_;
  const VectorXd y = z - z_pred;
  UpdateStateAndPosition(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  const VectorXd z_pred = GetPredictedStateForRadar();
  VectorXd y = z - z_pred;
  NormalizeAngle(y(1));
  UpdateStateAndPosition(y);
}

void KalmanFilter::UpdateStateAndPosition(const VectorXd &y){
  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  const MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ -= K * H_ * P_;
}

Eigen::VectorXd KalmanFilter::GetPredictedStateForRadar(){
        VectorXd z_pred;
        const float px = x_(0);
        const float py = x_(1);
        const float vx = x_(2);
        const float vy = x_(3);
        const float rho = sqrt(px * px + py * py);
        float phi = atan2(py,px);
        float rho_dot = 0;
        if(fabs(rho) >= 0.0001){
           rho_dot = (px*vx + py*vy) / rho;
        }
        z_pred = VectorXd(3);
        z_pred << rho, phi, rho_dot;
        return z_pred;
}

void KalmanFilter::NormalizeAngle(double& phi)
{
    atan2(sin(phi), cos(phi));
}

