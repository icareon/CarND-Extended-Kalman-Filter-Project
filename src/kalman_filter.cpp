#include "kalman_filter.h"

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
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S_ = H_ * P_ * Ht + R_;
    MatrixXd Si = S_.inverse();
    MatrixXd K_ = P_ * Ht * Si;
    
    
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    float eps = 0.00001;
    if (px<eps && py<eps)
    {
        px = eps;
        py = eps;
        
    }
    else if (px<eps)
    {
        px = eps;
    }
    
    x_ = x_ + (K_* y);
    long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K_ * H_)*P_;

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
    
    
    float rho = sqrtf(px*px+py*py);
    if (rho<0.0001) {
        rho=0.0001;
    }
    
    //float eps = 0.0001;
    //if (px<eps && py<eps)
    //{
    //    px = eps;
    //    py = eps;
    //
    //}
    //else if (px<eps)
    //{
    //    px = eps;
    //}
   
    float phi = atan2f(py,px);
    float rho_dot = (px * vx + py *vy)/rho;
    
    VectorXd Hx(3);
    Hx<<rho,phi,rho_dot;
    
    MatrixXd Ht = H_.transpose();
    VectorXd y = z - Hx;
    y[1]=atan2f(sin(y[1]),cos(y[1]));
    MatrixXd S_ = H_ * P_ * Ht + R_;
    MatrixXd K_ = P_ * Ht * S_.inverse();
    long size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(size, size);
    
    x_ = x_ + K_ * y;
    P_ = (I_ - K_ * H_) * P_;
}
