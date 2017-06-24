#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth){

  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if(estimations.size()==0 or estimations.size()!=ground_truth.size()){
    cout<< "Invalid Input"<<endl;
    return rmse;
  }
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    rmse =rmse+ (ground_truth[i]-estimations[i]).array().pow(2).matrix();

  }

  //calculate the mean
  rmse=rmse.array()/estimations.size();
  //calculate the squared root
  rmse=rmse.array().sqrt();
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px_py_square = px*px+py*py;
  float l2_px_py=sqrt(px_py_square);
  //check division by zero
//  cout<<"CalculateJacobian() - Error - Division by Zero"<<endl;

  if(px_py_square<0.0001){
    cout<<"CalculateJacobian() - Error - Division by Zero"<<endl;
    Hj<< 0,0,0,0,
    0,0,0,0,
    0,0,0,0;
    return Hj;
  }
  //compute the Jacobian matrix
  Hj<< px/l2_px_py, py/l2_px_py, 0., 0.,
  -py/px_py_square, px/px_py_square, 0., 0.,
  py*(vx*py-vy*px)/pow(l2_px_py,3), px*(vy*px-vx*py)/pow(l2_px_py,3), px/l2_px_py, py/l2_px_py;
  return Hj;
}
