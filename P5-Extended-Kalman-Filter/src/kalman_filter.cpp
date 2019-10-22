#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    MatrixXd F_T = F_.transpose();
    P_ = F_ * P_*F_T + Q_ ;

}


void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    MatrixXd I;

    
    VectorXd y = z - H_*x_;

    MatrixXd H_T = H_.transpose();
    MatrixXd S = H_ * P_ * H_T + R_;
    MatrixXd S_inv = S.inverse();
    MatrixXd K = P_ * H_T*S_inv;

    x_ = x_ + (K * y);
  

    I = MatrixXd::Identity(4, 4);
  
    P_ = (I - K * H_)*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */


    MatrixXd I;


    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];

    float px2 = px * px;
    float py2 = py * py;


    float rho = sqrt(px2 + py2);
    float phi = atan2(py, px);

    float corrected_rho = rho;

    if (corrected_rho == 0)
        corrected_rho = 0.0001;

    float rho_dot = ((px * vx) + (py * vy)) / corrected_rho;

    VectorXd x_pred(3);
    x_pred << rho, phi, rho_dot;

    VectorXd y = z - x_pred;

    //if angle is more than 180 decrease it by 360 
    while (y(1) > M_PI) y(1) -= 2 * M_PI;

    //if angle is less than -180 increase it by 360
    while (y(1) < -M_PI) y(1) += 2 * M_PI;

    MatrixXd H_T = H_.transpose();
    MatrixXd S = H_ * P_ * H_T + R_;
    MatrixXd S_inv = S.inverse();
    MatrixXd K = P_ * H_T*S_inv;

    x_ = x_ + (K * y);
  
  
    I = MatrixXd::Identity(4, 4);
  
    P_ = (I - K * H_)*P_;

}
