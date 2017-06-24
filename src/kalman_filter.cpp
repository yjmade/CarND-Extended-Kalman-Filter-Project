#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_radar, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Rr = R_radar;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
//  Hj=tools:
  VectorXd z_pred = h_radar(x_);
  VectorXd y = z - z_pred;
  y[1]=normalizePhi(y[1]);
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + Rr;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
  
}

Eigen::VectorXd KalmanFilter::h_radar(Eigen::VectorXd &measurement){
  float px=measurement[0];
  float py=measurement[1];
  float vx=measurement[2];
  float vy=measurement[3];
  float rho=sqrt(px*px+py*py);
  float phi=atan2(py, px);
  float rhodot;
  if (fabs(rho)<0.0001){
    rhodot=0;
  }else{
    rhodot=(px*vx+py*vy)/rho;
  }
  Eigen::VectorXd new_measurement=VectorXd(3);
  new_measurement<< rho, phi, rhodot;
  return new_measurement;
}

float KalmanFilter::normalizePhi(float phi){
  if (phi>M_PI){
    return normalizePhi(phi-2*M_PI);
  }
  if(phi<-M_PI){
    return normalizePhi(phi+2*M_PI);
  }
  return phi;
}
