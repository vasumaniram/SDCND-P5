#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
     cout<<"The Estimations Size is Either Zero Or It Does Not Match With Ground Truth's Size";
    return rmse;
  }
  
  for(int i=0;i<estimations.size();++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual  = residual.array() * residual.array();
    rmse += residual;
  }
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  px_py = (px * px) + (py * py);
  root = sqrt(px_py);
  px_by_root = px/root;
  py_by_root = py/root;
  px_by_pxpy = px / px_py;
  py_by_pxpy = py / px_py;
  
  if(fbs(px_py) < 0.0001){
     cout<<"Divide By Zero Error"<<endl;
     return Hj;
  }
  Hj << px_by_root,py_by_root,0,0,
       -py_by_pxpy,px_by_pxpy,0,0,
       (py*(vx*py-vy*px))/((px_py)*root),(px*(vy*px-vx*py))/((px_py)*root),px_by_root,py_by_root;
  return Hj;
}
