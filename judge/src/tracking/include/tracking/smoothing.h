#ifndef SMOOTHING_H
#define SMOOTHING_H

#include <ros/ros.h>
#include "tracking/controller.h" // Include to use the ControlCommand struct

/**
 * @class OutputSmoother
 * @brief An output command smoother integrating a four-stage low-pass filter
 *        and a rate limiter.
 *
 * This class takes raw velocity commands and applies several layers of filtering
 * to produce a smoother, less jerky output suitable for a real vehicle.
 * The implementation is a direct translation of the provided smoothing.py.
 */
class OutputSmoother {
public:
    /**
     * @brief Constructor for the OutputSmoother.
     * @param nh ROS NodeHandle to load filter parameters from the server.
     */
    OutputSmoother(ros::NodeHandle& nh);

    /**
     * @brief Applies the multi-stage smoothing process to raw velocity commands.
     * @param raw_vel A ControlCommand struct containing the raw target velocities.
     * @param dt The time delta since the last call (in seconds).
     * @return A ControlCommand struct with the smoothed velocities.
     */
    ControlCommand smooth(const ControlCommand& raw_vel, double dt);

private:
    // --- Filter Parameters ---
    // Alpha values for the four low-pass filters
    double lpf_alpha_1_;
    double lpf_alpha_2_;
    double lpf_alpha_3_;
    double lpf_alpha_4_;

    // Maximum change rates for the rate limiter
    double rate_limit_xyz_; // for x, y, z axes in m/s^2
    double rate_limit_yaw_; // for yaw axis in rad/s^2

    // --- Filter State Variables ---
    // We reuse the ControlCommand struct to hold the state of each filter stage
    ControlCommand smooth_vel_lpf1_;
    ControlCommand smooth_vel_lpf2_;
    ControlCommand smooth_vel_lpf3_;
    ControlCommand smooth_vel_lpf4_;
    ControlCommand smooth_vel_final_; // Output of the rate limiter
};

#endif // SMOOTHING_H