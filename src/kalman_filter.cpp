#include "kalman_filter.h"
#include <iostream>
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::atan2;

#define PI 3.14159265

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
  VectorXd z_pred = getPredictedState("LASER");
  Update(z,z_pred);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd z_pred = getPredictedState("RADAR");
  Update(z,z_pred);
}

void KalmanFilter::Update(const VectorXd &z,const VectorXd &z_pred){
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::getPredictedState(const std::string sensorType){
  VectorXd z_pred;
    if (sensorType == "RADAR"){
        float px = x_(0);
        float py = x_(1);
        float vx = x_(2);
        float vy = x_(3);
        float rho = sqrt(px * px + py * py);
        float phi = atan2(py,px);
        float rho_dot = 0;
        if(fabs(rho) >= 0.0001){
           rho_dot = (px*vx + py*vy) / rho;
        }
        z_pred = VectorXd(3);
        z_pred << rho, phi, rho_dot;
    } else if(sensorType == "LASER"){
        z_pred = H_ * x_;
    }
    return z_pred;
}

