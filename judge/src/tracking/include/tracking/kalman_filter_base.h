#ifndef KALMAN_FILTER_BASE_H
#define KALMAN_FILTER_BASE_H

#include <Eigen/Dense>

/**
 * @class KalmanFilterBase
 * @brief A standard, generic implementation of a linear Kalman Filter.
 *
 * This class provides the core predict and update steps of a Kalman Filter.
 * It is designed to be configured and used by other classes, such as the
 * Interacting Multiple Model (IMM) filter.
 */
class KalmanFilterBase {
public:
    /**
     * @brief Constructor.
     * @param dim_x The dimension of the state vector.
     * @param dim_z The dimension of the measurement vector.
     */
    KalmanFilterBase(int dim_x, int dim_z);

    /**
     * @brief Initializes the filter's state and covariance.
     * @param x_init Initial state vector.
     * @param P_init Initial covariance matrix.
     */
    void init(const Eigen::VectorXd& x_init, const Eigen::MatrixXd& P_init);

    /**
     * @brief Performs the prediction step of the Kalman Filter.
     * x = F * x
     * P = F * P * F' + Q
     */
    void predict();

    /**
     * @brief Performs the update step of the Kalman Filter.
     * @param z The measurement vector.
     */
    void update(const Eigen::VectorXd& z);

    // --- Public Member Variables ---
    // For easy configuration by the owner (e.g., IMMFilter)

    // State and Covariance
    Eigen::VectorXd x;      // State estimate vector
    Eigen::MatrixXd P;      // Estimate covariance matrix

    // Model Matrices
    Eigen::MatrixXd F;      // State transition matrix
    Eigen::MatrixXd H;      // Measurement matrix
    Eigen::MatrixXd Q;      // Process noise covariance matrix
    Eigen::MatrixXd R;      // Measurement noise covariance matrix

    // Internal variables (public for inspection)
    Eigen::VectorXd y;      // Innovation or residual
    Eigen::MatrixXd S;      // Innovation (or residual) covariance
    Eigen::MatrixXd K;      // Kalman gain

private:
    int dim_x_;             // State vector dimension
    int dim_z_;             // Measurement vector dimension
    Eigen::MatrixXd I_;     // Identity matrix
};

#endif // KALMAN_FILTER_BASE_H