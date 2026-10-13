#include "tracking/smoothing.h"
#include <algorithm> // For std::max/min used in clamp
#include <cmath>

// Helper function for C++14 or older that don't have std::clamp
// Re-defined here to keep the file self-contained.
template<class T>
constexpr const T& clamp_val( const T& v, const T& lo, const T& hi ) {
    return std::max(lo, std::min(v, hi));
}

OutputSmoother::OutputSmoother(ros::NodeHandle& nh) {
    // Load parameters from the ROS Parameter Server.
    // Default values are taken directly from the Python source code.
    nh.param("smoother/lpf_alpha_1", lpf_alpha_1_, 0.4); // Example value, adjust as needed
    nh.param("smoother/lpf_alpha_2", lpf_alpha_2_, 0.3); // Example value, adjust as needed
    nh.param("smoother/lpf_alpha_3", lpf_alpha_3_, 0.2); // Example value, adjust as needed
    nh.param("smoother/lpf_alpha_4", lpf_alpha_4_, 0.1); // Example value, adjust as needed

    nh.param("smoother/rate_limit_xyz", rate_limit_xyz_, 2.0); // Example value, m/s^2
    nh.param("smoother/rate_limit_yaw", rate_limit_yaw_, 1.5); // Example value, rad/s^2

    // The ControlCommand member variables are automatically default-initialized to all zeros,
    // which is the correct initial state.
}

ControlCommand OutputSmoother::smooth(const ControlCommand& raw_vel, double dt) {
    // --- Stage 1: First Low-Pass Filter ---
    smooth_vel_lpf1_.x = lpf_alpha_1_ * raw_vel.x + (1.0 - lpf_alpha_1_) * smooth_vel_lpf1_.x;
    smooth_vel_lpf1_.y = lpf_alpha_1_ * raw_vel.y + (1.0 - lpf_alpha_1_) * smooth_vel_lpf1_.y;
    smooth_vel_lpf1_.z = lpf_alpha_1_ * raw_vel.z + (1.0 - lpf_alpha_1_) * smooth_vel_lpf1_.z;
    smooth_vel_lpf1_.yaw = lpf_alpha_1_ * raw_vel.yaw + (1.0 - lpf_alpha_1_) * smooth_vel_lpf1_.yaw;

    // --- Stage 2: Second Low-Pass Filter (input is the output of stage 1) ---
    smooth_vel_lpf2_.x = lpf_alpha_2_ * smooth_vel_lpf1_.x + (1.0 - lpf_alpha_2_) * smooth_vel_lpf2_.x;
    smooth_vel_lpf2_.y = lpf_alpha_2_ * smooth_vel_lpf1_.y + (1.0 - lpf_alpha_2_) * smooth_vel_lpf2_.y;
    smooth_vel_lpf2_.z = lpf_alpha_2_ * smooth_vel_lpf1_.z + (1.0 - lpf_alpha_2_) * smooth_vel_lpf2_.z;
    smooth_vel_lpf2_.yaw = lpf_alpha_2_ * smooth_vel_lpf1_.yaw + (1.0 - lpf_alpha_2_) * smooth_vel_lpf2_.yaw;

    // --- Stage 3: Third Low-Pass Filter (input is the output of stage 2) ---
    smooth_vel_lpf3_.x = lpf_alpha_3_ * smooth_vel_lpf2_.x + (1.0 - lpf_alpha_3_) * smooth_vel_lpf3_.x;
    smooth_vel_lpf3_.y = lpf_alpha_3_ * smooth_vel_lpf2_.y + (1.0 - lpf_alpha_3_) * smooth_vel_lpf3_.y;
    smooth_vel_lpf3_.z = lpf_alpha_3_ * smooth_vel_lpf2_.z + (1.0 - lpf_alpha_3_) * smooth_vel_lpf3_.z;
    smooth_vel_lpf3_.yaw = lpf_alpha_3_ * smooth_vel_lpf2_.yaw + (1.0 - lpf_alpha_3_) * smooth_vel_lpf3_.yaw;

    // --- Stage 4: Fourth Low-Pass Filter (input is the output of stage 3) ---
    smooth_vel_lpf4_.x = lpf_alpha_4_ * smooth_vel_lpf3_.x + (1.0 - lpf_alpha_4_) * smooth_vel_lpf4_.x;
    smooth_vel_lpf4_.y = lpf_alpha_4_ * smooth_vel_lpf3_.y + (1.0 - lpf_alpha_4_) * smooth_vel_lpf4_.y;
    smooth_vel_lpf4_.z = lpf_alpha_4_ * smooth_vel_lpf3_.z + (1.0 - lpf_alpha_4_) * smooth_vel_lpf4_.z;
    smooth_vel_lpf4_.yaw = lpf_alpha_4_ * smooth_vel_lpf3_.yaw + (1.0 - lpf_alpha_4_) * smooth_vel_lpf4_.yaw;

    // --- Stage 5: Rate Limiter (input is the output of stage 4) ---
    double max_change_xyz = rate_limit_xyz_ * dt;
    double max_change_yaw = rate_limit_yaw_ * dt;

    // X axis
    double change_x = clamp_val(smooth_vel_lpf4_.x - smooth_vel_final_.x, -max_change_xyz, max_change_xyz);
    smooth_vel_final_.x += change_x;

    // Y axis
    double change_y = clamp_val(smooth_vel_lpf4_.y - smooth_vel_final_.y, -max_change_xyz, max_change_xyz);
    smooth_vel_final_.y += change_y;

    // Z axis
    double change_z = clamp_val(smooth_vel_lpf4_.z - smooth_vel_final_.z, -max_change_xyz, max_change_xyz);
    smooth_vel_final_.z += change_z;

    // Yaw axis
    double change_yaw = clamp_val(smooth_vel_lpf4_.yaw - smooth_vel_final_.yaw, -max_change_yaw, max_change_yaw);
    smooth_vel_final_.yaw += change_yaw;
    
    // The Python code also returns a copy of the final smoothed velocity.
    return smooth_vel_final_;
}