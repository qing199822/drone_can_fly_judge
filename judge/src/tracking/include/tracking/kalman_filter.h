#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "tracking/kalman_filter_base.h"
#include <vector>
#include <string>
#include <ros/ros.h>

/**
 * @class IMMFilter
 * @brief Implements an Interacting Multiple Model (IMM) filter.
 *
 * This class manages three motion models (Constant Velocity, Constant Acceleration,
 * and High Maneuver) to provide a robust state estimate for a tracked target.
 * It uses a Bayesian approach to update the probability of each model being correct.
 * It also includes a four-stage low-pass filter on its output to provide
 * smooth estimates to the controller, mirroring the Python implementation.
 */
class IMMFilter {
public:
    /**
     * @brief Constructor.
     * @param dt Time step.
     * @param u_init Initial u-coordinate (pixel).
     * @param v_init Initial v-coordinate (pixel).
     * @param s_init Initial area (pixels^2).
     */
    IMMFilter(double dt, double u_init, double v_init, double s_init);

    /**
     * @brief Predicts the state of each internal filter.
     */
    void predict();

    /**
     * @brief Updates the filter with a new measurement.
     * @param z The measurement vector [u, v, s].
     */
    void update(const Eigen::VectorXd& z);

    /**
     * @brief Resets the filter to an initial state.
     * @param u_init Initial u-coordinate.
     * @param v_init Initial v-coordinate.
     * @param s_init Initial area.
     */
    void reset(double u_init, double v_init, double s_init);

    /**
     * @brief Gets the final, smoothed 5D state estimate.
     * @return A 5D vector [u, v, du, dv, s].
     */
    Eigen::VectorXd getState() const;

    // --- Public Members for Inspection ---
    Eigen::VectorXd x; // Fused state estimate [u, v, du, dv, ddu, ddv, s]
    Eigen::MatrixXd P; // Fused covariance estimate
    Eigen::VectorXd mu; // Model probabilities [CV, CA, HM]

private:
    void setupFilters(double dt, double u_init, double v_init, double s_init);
    void applyLowPassFilter();

    // --- Core IMM Algorithm Steps ---
    void computeMixingProbabilities();
    void mixStates();
    void updateLikelihoods();
    void updateModelProbabilities();
    void fuseStateAndCovariance();

    // --- Filter Configuration ---
    static constexpr int NUM_MODELS = 3;
    const int dim_x_ = 7; // [u, v, du, dv, ddu, ddv, s]
    const int dim_z_ = 3; // [u, v, s]

    // --- IMM Members ---
    std::vector<KalmanFilterBase> filters_;
    std::vector<double> likelihoods_;
    Eigen::MatrixXd M_;             // Markov transition matrix
    Eigen::MatrixXd omega_;         // Mixing probabilities
    Eigen::RowVectorXd c_bar_;

    std::vector<std::string> model_names_ = {"CV", "CA", "HM"};
    int current_mode_idx_ = 0;

    // --- Low-Pass Filter Members ---
    double lpf_alpha_ = 0.2; // Same as Python version
    Eigen::VectorXd lpf1_state_;
    Eigen::VectorXd lpf2_state_;
    Eigen::VectorXd lpf3_state_;
    mutable Eigen::VectorXd lpf4_state_; // Mutable to be changed in const getState()
};

#endif // KALMAN_FILTER_H