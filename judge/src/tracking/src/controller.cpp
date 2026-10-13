#include "tracking/controller.h"
#include <cmath>
#include <algorithm> // For std::clamp
#include <utility>   // For std::pair

// Helper function for C++14 or older that don't have std::clamp
template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi ) {
    return std::max(lo, std::min(v, hi));
}


TrackingController::TrackingController(ros::NodeHandle& nh) {
    // Load parameters from the ROS Parameter Server with defaults from the Python code
    nh.param("controller/Kp_pos_y", Kp_pos_y_, 0.0025);
    nh.param("controller/Kd_pos_y", Kd_pos_y_, 0.001);
    nh.param("controller/Kp_yaw", Kp_yaw_, -0.0045);
    nh.param("controller/Kd_yaw", Kd_yaw_, -0.002);
    nh.param("controller/Kp_z_altitude", Kp_z_altitude_, 1.0);

    nh.param("controller/max_speed_x", MAX_SPEED_X_, 7.0);
    nh.param("controller/max_backward_speed", MAX_BACKWARD_SPEED_, 2.0);
    nh.param("controller/max_speed_y", MAX_SPEED_Y_, 7.0);
    nh.param("controller/max_speed_z", MAX_SPEED_Z_, 1.5);
    nh.param("controller/max_speed_yaw", MAX_SPEED_YAW_, 1.0);

    nh.param("controller/height_speed_factor", HEIGHT_SPEED_FACTOR_, 1.5);
    nh.param("controller/min_speed_limit", MIN_SPEED_LIMIT_, 2.0);

    nh.param("controller/use_pitch_backward", USE_PITCH_BACKWARD_, true);
    nh.param("controller/max_backward_pitch", MAX_BACKWARD_PITCH_, -0.5);
    nh.param("controller/pitch_to_velocity_ratio", PITCH_TO_VELOCITY_RATIO_, 0.2);
    nh.param("controller/throttle_reduction_ratio", THROTTLE_REDUCTION_RATIO_, 0.45);
    nh.param("controller/backward_threshold", BACKWARD_THRESHOLD_, -0.5);
    nh.param("controller/backward_transition_start", BACKWARD_TRANSITION_START_, -0.5);
    
    nh.param("controller/velocity_suppression_enabled", VELOCITY_SUPPRESSION_ENABLED_, true);
    // 距离保持开关：false 时追踪只做居中/高度控制，不根据目标框面积前进或后退。
    nh.param("controller/enable_distance_control", ENABLE_DISTANCE_CONTROL_, true);
    nh.param("controller/max_suppression_factor", MAX_SUPPRESSION_FACTOR_, 0.6);
    nh.param("controller/suppression_velocity_range", SUPPRESSION_VELOCITY_RANGE_, 1.5);

    nh.param("controller/activator/base_sensitivity", base_sensitivity_, 0.3);
    nh.param("controller/activator/max_additional_sensitivity", max_additional_sensitivity_, 0.7);
    nh.param("controller/activator/scaling_factor", scaling_factor_, 0.01);
    nh.param("controller/activator/gain_multiplier", gain_multiplier_, 1.8);
    nh.param("controller/activator/exp_factor", exp_factor_, 0.8);
    nh.param("controller/activator/speed_retention_factor", speed_retention_factor_, 0.8);

    // These parameters are shared with the state machine but needed here for calculations
    nh.param("state_machine/target_bbox_area", TARGET_BBOX_AREA_, 2700.0);
    nh.param("state_machine/transition_duration", TRANSITION_DURATION_, 1.5);

    max_speed_x_ = MAX_SPEED_X_; // Initialize dynamic max speed
}

ControlCommand TrackingController::calculate_tracking_commands(const Eigen::VectorXd& kf_state,
                                                                 double height, double target_height,
                                                                 const geometry_msgs::Twist& current_velocity_body,
                                                                 double u_center, double v_center)
{
    double u_est = kf_state(0);
    double v_est = kf_state(1);
    double du_est = kf_state(2);
    double dv_est = kf_state(3);
    double s_est = kf_state(4);

    // --- 1. Initialization ---
    auto sensitivity_pair = calculate_dynamic_sensitivity(du_est, dv_est);
    double sensitivity = sensitivity_pair.first;
    ros::Time current_time = ros::Time::now();

    // --- 2. Cruise -> Tracking Transition Logic ---
    bool is_in_transition = false;
    double transition_factor = 0.0;
    if (was_cruising_ && !cruise_entry_time_.is_zero()) {
        double time_since_switch = (current_time - cruise_entry_time_).toSec();
        if (time_since_switch < TRANSITION_DURATION_) {
            is_in_transition = true;
            transition_factor = std::max(0.0, 1.0 - time_since_switch / TRANSITION_DURATION_);
        } else {
            was_cruising_ = false;
        }
    }

    // --- 3. Calculate Forward Velocity (vx) ---
    double norm_error;
    if (is_in_transition) {
        norm_error = (s_est - TARGET_BBOX_AREA_) / (TARGET_BBOX_AREA_ * 0.5);
    } else {
        norm_error = (s_est - TARGET_BBOX_AREA_) / TARGET_BBOX_AREA_;
    }
    norm_error = clamp(norm_error, -2.0, 2.0);

    max_speed_x_ = calculate_max_speed_limit(height);

    auto activator_output = enhanced_activator(norm_error, current_velocity_body.linear.x, max_speed_x_, sensitivity, is_in_transition);
    double base_output = activator_output.first;
    adaptive_gain_ = activator_output.second; // Store for debugging

    double calculated_vel_x = -base_output;
    double desired_vel_x;

    if (is_in_transition) {
        double blended_speed = (transition_factor * cruise_entry_speed_ + (1.0 - transition_factor) * calculated_vel_x);
        desired_vel_x = clamp(blended_speed, -max_speed_x_, max_speed_x_);
    } else {
        desired_vel_x = calculated_vel_x;
    }

    // 禁用距离控制：不前进也不后退，只靠 y/yaw 把目标放回画面中央。
    if (!ENABLE_DISTANCE_CONTROL_) {
        desired_vel_x = 0.0;
        is_in_transition = false;
    }

    // 后退速度单独限幅：后退太猛容易让目标从相机视野下方/边缘丢失，
    // 所以后退最大速度比前进最大速度更小。
    if (desired_vel_x < -MAX_BACKWARD_SPEED_) {
        desired_vel_x = -MAX_BACKWARD_SPEED_;
    }

    // --- 4. Calculate Lateral, Yaw, and Altitude Velocities ---
    double u_error = u_est - u_center;
    double vel_y = -(Kp_pos_y_ * u_error + Kd_pos_y_ * du_est);
    double vel_yaw = Kp_yaw_ * u_error + Kd_yaw_ * du_est;
    double vel_z = Kp_z_altitude_ * (target_height - height);

    ControlCommand commands;
    commands.mode = is_in_transition ? "Trans" : "Track";
    commands.desired_vel_x = desired_vel_x;

    // --- 5. Pitch Backward Strategy ---
    if (USE_PITCH_BACKWARD_ && desired_vel_x < BACKWARD_THRESHOLD_) {
        // Full pitch backward for significant reverse speed
        commands.pitch_angle = calculate_backward_pitch(desired_vel_x, height);
        commands.throttle_reduction = calculate_throttle_reduction(commands.pitch_angle);
        double adjusted_vel_z = vel_z + commands.throttle_reduction;
        
        commands.x = 0.0; // Velocity control is replaced by pitch
        commands.y = clamp(vel_y, -MAX_SPEED_Y_, MAX_SPEED_Y_);
        commands.z = clamp(adjusted_vel_z, -MAX_SPEED_Z_, MAX_SPEED_Z_);
        commands.yaw = clamp(vel_yaw, -MAX_SPEED_YAW_, MAX_SPEED_YAW_);
        commands.pitch = commands.pitch_angle;
        commands.mode = "PitchBackward";

    } else if (USE_PITCH_BACKWARD_ && desired_vel_x < BACKWARD_TRANSITION_START_) {
        // Smooth transition to pitch backward
        double transition_ratio = (desired_vel_x - BACKWARD_TRANSITION_START_) / (BACKWARD_THRESHOLD_ - BACKWARD_TRANSITION_START_);
        transition_ratio = clamp(transition_ratio, 0.0, 1.0);

        commands.pitch_angle = calculate_backward_pitch(desired_vel_x, height) * transition_ratio;
        commands.throttle_reduction = calculate_throttle_reduction(commands.pitch_angle);
        
        double mixed_vel_x = desired_vel_x * (1.0 - transition_ratio);
        double adjusted_vel_z = vel_z + commands.throttle_reduction;

        commands.x = clamp(mixed_vel_x, -MAX_SPEED_X_, MAX_SPEED_X_);
        commands.y = clamp(vel_y, -MAX_SPEED_Y_, MAX_SPEED_Y_);
        commands.z = clamp(adjusted_vel_z, -MAX_SPEED_Z_, MAX_SPEED_Z_);
        commands.yaw = clamp(vel_yaw, -MAX_SPEED_YAW_, MAX_SPEED_YAW_);
        commands.pitch = commands.pitch_angle;
        commands.mode = "MixedBackward";
        // 2026-09-24：打印前向/偏航分量，便于离线判断"前进被谁砍、yaw 是否在限幅"。
        ROS_INFO_THROTTLE(1.0,
                          "PD Track err=%.2f gain=%.2f -> vx=%.2f | u_err=%.0f du=%.0f vy=%.2f vyaw=%.2f",
                          norm_error, adaptive_gain_,
                          commands.x, u_error, du_est, commands.y, commands.yaw);

    } else {
        // Standard forward or minor backward velocity control
        commands.x = clamp(desired_vel_x, -MAX_SPEED_X_, MAX_SPEED_X_);
        commands.y = clamp(vel_y, -MAX_SPEED_Y_, MAX_SPEED_Y_);
        commands.z = clamp(vel_z, -MAX_SPEED_Z_, MAX_SPEED_Z_);
        commands.yaw = clamp(vel_yaw, -MAX_SPEED_YAW_, MAX_SPEED_YAW_);
        commands.pitch = 0.0;
        // 2026-09-24：同上，标准前向分支也打印前向/偏航分量。
        ROS_INFO_THROTTLE(1.0,
                          "PD Track err=%.2f gain=%.2f -> vx=%.2f | u_err=%.0f du=%.0f vy=%.2f vyaw=%.2f",
                          norm_error, adaptive_gain_,
                          commands.x, u_error, du_est, commands.y, commands.yaw);
    }
    
    // Populate debug fields
    commands.transition_factor = transition_factor;
    commands.sensitivity = sensitivity;
    commands.adaptive_gain = adaptive_gain_;
    commands.max_speed_limit = max_speed_x_;

    // --- 6. Small Velocity Suppression ---
    ControlCommand final_commands = apply_small_velocity_suppression(commands, current_velocity_body.linear.x);

    // --- Debugging Info ---
    debug_counter_++;
    if (debug_counter_ % debug_interval_ == 0) {
        ROS_INFO("PD Track: err=%.2f, gain=%.2f -> vx=%.2f", norm_error, adaptive_gain_, desired_vel_x);
        ROS_INFO("PD Yaw: u_err=%.1f, du_est=%.1f -> vyaw=%.2f", u_error, du_est, vel_yaw);
        if (final_commands.pitch != 0) {
            ROS_INFO("Pitch Backward: pitch=%.3f, throttle_red=%.3f, mode=%s",
                     final_commands.pitch, final_commands.throttle_reduction, final_commands.mode.c_str());
        }
    }

    return final_commands;
}

ControlCommand TrackingController::apply_small_velocity_suppression(ControlCommand commands, double current_velocity_x)
{
    // 距离控制已禁用：x 恒为 0，不参与抑制/死区处理。
    if (!ENABLE_DISTANCE_CONTROL_) {
        last_velocity_x_ = current_velocity_x;
        last_command_x_ = commands.x;
        return commands;
    }

    // 后退指令（x < 0）不进行抑制/死区处理，确保目标靠近时能果断后退拉开距离。
    if (commands.x < 0.0) {
        last_velocity_x_ = current_velocity_x;
        last_command_x_ = commands.x;
        return commands;
    }

    // This logic is directly translated from Python, maintaining the suppression algorithm.
    double current_vel = commands.x;
    double last_vel = last_command_x_;
    
    if (std::abs(current_vel) <= 1.5 && std::abs(current_velocity_x) <= 1.5) {
        if (std::abs(current_vel) <= 1.0) {
            double velocity_change = current_vel - last_vel;
            if (std::abs(velocity_change) < 0.15) {
                commands.x = last_vel;
                if (debug_counter_ % debug_interval_ == 0 && std::abs(velocity_change) > 0.02) {
                    ROS_INFO("Deadzone active: change %.3f < threshold 0.15, holding %.2f", velocity_change, last_vel);
                }
                last_velocity_x_ = current_velocity_x;
                last_command_x_ = commands.x;
                return commands;
            }
        }

        double vel_change = std::abs(current_vel) - std::abs(last_vel);
        if (vel_change < 0) {
            double suppression_factor = 1.0 - (std::abs(current_vel) / 1.5) * 0.6;
            double smoothed_vel = last_vel + (current_vel - last_vel) * suppression_factor;
            commands.x = smoothed_vel;

            if (debug_counter_ % debug_interval_ == 0 && std::abs(vel_change) > 0.1) {
                ROS_INFO("Velocity suppression: %.2f -> %.2f -> %.2f (Factor: %.2f)",
                         last_vel, current_vel, smoothed_vel, suppression_factor);
            }
        }
    }

    last_velocity_x_ = current_velocity_x;
    last_command_x_ = commands.x;
    return commands;
}

double TrackingController::calculate_throttle_reduction(double pitch_angle) {
    double base_reduction = std::abs(pitch_angle) * THROTTLE_REDUCTION_RATIO_;
    
    double nonlinear_threshold = 0.15;
    double nonlinear_slope = 0.2;
    
    double additional_reduction = 0.0;
    if (std::abs(pitch_angle) > nonlinear_threshold) {
        additional_reduction = nonlinear_slope * (std::abs(pitch_angle) - nonlinear_threshold);
    }
    
    double reduction = base_reduction + additional_reduction;
    
    if (debug_counter_ % debug_interval_ == 0 && std::abs(pitch_angle) > 0.1) {
        ROS_INFO("Throttle Comp: pitch=%.3f, base=%.3f, additional=%.3f, total=%.3f",
                 pitch_angle, base_reduction, additional_reduction, reduction);
    }
    
    return -reduction;
}

std::pair<double, double> TrackingController::calculate_dynamic_sensitivity(double du_est, double dv_est) {
    double target_speed_px = std::sqrt(du_est * du_est + dv_est * dv_est);
    double sensitivity = base_sensitivity_ + max_additional_sensitivity_ / (1.0 + std::exp(-scaling_factor_ * target_speed_px));
    return {sensitivity, target_speed_px};
}

std::pair<double, double> TrackingController::enhanced_activator(double error, double current_speed, double max_speed, double sensitivity, bool is_cruising_transition) {
    double adaptive_gain = gain_multiplier_ * sensitivity / (1.0 + std::exp(-exp_factor_ * std::abs(error)));
    double base_output = max_speed * std::tanh(adaptive_gain * error);
    
    if (is_cruising_transition && error < -0.5) {
        // Logic for cruise speed retention is maintained, though its effect is blended in the main function.
        double cruise_speed_floor = std::min(std::abs(current_speed) * speed_retention_factor_, max_speed);
        // The Python code returns this but doesn't seem to use it, so we just calculate and return the primary values.
    }
    
    return {base_output, adaptive_gain};
}

double TrackingController::calculate_max_speed_limit(double height) {
    if (height <= 0) {
        return MAX_SPEED_X_ / 2.0;
    }
    double height_based_limit = HEIGHT_SPEED_FACTOR_ * height;
    double speed_limit = std::min(MAX_SPEED_X_, height_based_limit);
    speed_limit = std::max(speed_limit, MIN_SPEED_LIMIT_);
    return speed_limit;
}

double TrackingController::calculate_backward_pitch(double desired_vel_x, double height) {
    double base_pitch = desired_vel_x * PITCH_TO_VELOCITY_RATIO_;
    double height_factor = clamp(height / 2.5, 0.5, 1.0);
    double pitch_angle = base_pitch * height_factor;
    
    pitch_angle = std::max(pitch_angle, MAX_BACKWARD_PITCH_);
    pitch_angle = std::min(pitch_angle, -0.1); // Minimum pitch down
    
    return pitch_angle;
}

void TrackingController::set_cruise_transition(const geometry_msgs::Twist& current_velocity_body, double cruise_speed_threshold) {
    if (std::abs(current_velocity_body.linear.x) > cruise_speed_threshold) {
        was_cruising_ = true;
        cruise_entry_speed_ = std::abs(current_velocity_body.linear.x);
        cruise_entry_time_ = ros::Time::now();
    } else {
        was_cruising_ = false;
    }
}


// Note: The dash commands are simplified here as some Python dependencies (like bbox) are handled in the state machine.
// The state machine will now determine the u_error and pass it in.
ControlCommand TrackingController::calculate_dash_commands(bool target_visible, double u_error,
                                                           double height, double dash_initial_height,
                                                           double u_center, double v_center)
{
    // Dash parameters will be loaded from param server in a real implementation
    double DASH_SPEED = 5.0; 

    ControlCommand commands;
    commands.x = DASH_SPEED;
    commands.y = 0;
    commands.z = 0;
    commands.yaw = 0;
    commands.pitch = 0.0;

    // Height control
    double height_error = dash_initial_height - height;
    commands.z = clamp(Kp_z_altitude_ * height_error, -MAX_SPEED_Z_, MAX_SPEED_Z_);

    // Lateral and yaw control if target is visible
    if (target_visible) {
        commands.y = clamp(-0.002 * u_error, -1.5, 1.5);
        commands.yaw = clamp(Kp_yaw_ * u_error, -0.8, 0.8);
    }
    
    return commands;
}