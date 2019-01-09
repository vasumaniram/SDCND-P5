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
  cout<<"IN PREDICT"<<endl;
  cout<<"X_ "<<endl;
  std::cout << x_ << std::endl;
  cout<<"F_ "<<endl;
  std::cout << F_ << std::endl;
  cout<<"P_ "<<endl;
  std::cout << P_ << std::endl;
  cout<<"FT_ "<<endl;
  std::cout << F_.transpose() << std::endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  cout<<"IN UPDATE"<<endl;
  cout<<"X_ "<<endl;
  std::cout << x_ << std::endl;
  cout<<"H_ "<<endl;
  std::cout << H_ << std::endl;
  
  VectorXd z_pred = H_ * x_;
  
  cout<<"Z_ "<<endl;
  std::cout << z << std::endl;
  cout<<"z_pred_ "<<endl;
  std::cout << z_pred << std::endl;
  
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  cout<<"Ht "<<endl;
  std::cout << Ht << std::endl;
  cout<<"P_ "<<endl;
  std::cout << P_ << std::endl;
  cout<<"R_ "<<endl;
  std::cout << R_ << std::endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  cout<<"S "<<endl;
  std::cout << S << std::endl;
  cout<<"Si "<<endl;
  std::cout << Si << std::endl;
  MatrixXd PHt = P_ * Ht;
  cout<<"PHt "<<endl;
  std::cout << PHt << std::endl;
  MatrixXd K = PHt * Si;
  cout<<"K "<<endl;
  std::cout << K << std::endl;
  //new estimate
  cout<<"y "<<endl;
  std::cout << y << std::endl;
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  cout<<"IN UPDATE-EKF"<<endl;
  
  VectorXd z_pred = VectorXd(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float rho = sqrt(px * px + py * py);
  float phi = atan2(py,px) * 180 / PI;
  float rho_dot = (px*vx + py*vy) / sqrt(px * px + py * py);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  std::cout << y << std::endl;
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
