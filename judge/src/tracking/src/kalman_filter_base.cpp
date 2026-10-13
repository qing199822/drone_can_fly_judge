#include "tracking/kalman_filter_base.h"

KalmanFilterBase::KalmanFilterBase(int dim_x, int dim_z)
    : dim_x_(dim_x), dim_z_(dim_z) {
    x = Eigen::VectorXd::Zero(dim_x_);
    P = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    F = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    H = Eigen::MatrixXd::Zero(dim_z_, dim_x_);
    Q = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    R = Eigen::MatrixXd::Identity(dim_z_, dim_z_);
    I_ = Eigen::MatrixXd::Identity(dim_x_, dim_x_);

    y = Eigen::VectorXd::Zero(dim_z_);
    S = Eigen::MatrixXd::Zero(dim_z_, dim_z_);
    K = Eigen::MatrixXd::Zero(dim_x_, dim_z_);
}

void KalmanFilterBase::init(const Eigen::VectorXd& x_init, const Eigen::MatrixXd& P_init) {
    x = x_init;
    P = P_init;
}

void KalmanFilterBase::predict() {
    // Project the state ahead
    x = F * x;
    // Project the error covariance ahead
    P = F * P * F.transpose() + Q;
}

void KalmanFilterBase::update(const Eigen::VectorXd& z) {
    // Measurement residual (innovation)
    y = z - H * x;

    // Innovation covariance
    S = H * P * H.transpose() + R;

    // Optimal Kalman gain
    K = P * H.transpose() * S.inverse();

    // Updated (a posteriori) state estimate
    x = x + K * y;

    // Updated (a posteriori) estimate covariance
    P = (I_ - K * H) * P;
}