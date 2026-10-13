#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <Eigen/Dense> // <--- 修正点: 添加此行

/**
 * @struct ControlCommand
 * @brief A structure to hold the output of the controller calculations.
 * Replaces the Python dictionary for type safety and efficiency.
 */
struct ControlCommand {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;

    // Debugging and state information
    std::string mode;
    double transition_factor = 0.0;
    double sensitivity = 0.0;
    double adaptive_gain = 0.0;
    double max_speed_limit = 0.0;
    double pitch_angle = 0.0;
    double throttle_reduction = 0.0;
    double desired_vel_x = 0.0;
};


/**
 * @class TrackingController
 * @brief Responsible for generating control commands to track a target.
 * This class implements the core PD control logic, pitch-backward strategy,
 * and dynamic speed adjustments based on Kalman Filter state estimates.
 */
class TrackingController {
public:
    /**
     * @brief Constructor for the TrackingController.
     * @param nh A ROS NodeHandle to fetch parameters from the parameter server.
     */
    TrackingController(ros::NodeHandle& nh);

    /**
     * @brief Calculates the control commands for the tracking state.
     * @param kf_state The 5D state from the Kalman Filter [u, v, du, dv, s].
     * @param height Current altitude of the drone.
     * @param target_height The desired altitude.
     * @param current_velocity_body The drone's current velocity in its body frame.
     * @param u_center The horizontal center of the camera frame.
     * @param v_center The vertical center of the camera frame.
     * @return A ControlCommand struct containing the calculated velocities and pitch.
     */
    ControlCommand calculate_tracking_commands(const Eigen::VectorXd& kf_state,
                                                 double height, double target_height,
                                                 const geometry_msgs::Twist& current_velocity_body,
                                                 double u_center = 320.0, double v_center = 240.0);
    
    /**
     * @brief Calculates control commands for the high-speed dash state.
     * @param locked_target_bbox Optional bounding box of the target.
     * @param height Current altitude of the drone.
     * @param dash_initial_height Altitude at the start of the dash.
     * @param u_center The horizontal center of the camera frame.
     * @param v_center The vertical center of the camera frame.
     * @return A ControlCommand struct for dashing.
     */
    ControlCommand calculate_dash_commands(bool target_visible, double u_error,
                                           double height, double dash_initial_height,
                                           double u_center = 320.0, double v_center = 240.0);


    /**
     * @brief Informs the controller that a transition from cruising has started.
     * @param current_velocity_body The drone's velocity at the moment of transition.
     */
    void set_cruise_transition(const geometry_msgs::Twist& current_velocity_body, double cruise_speed_threshold);


private:
    // --- Helper Calculation Methods ---
    ControlCommand apply_small_velocity_suppression(ControlCommand commands, double current_velocity_x);
    double calculate_throttle_reduction(double pitch_angle);
    std::pair<double, double> calculate_dynamic_sensitivity(double du_est, double dv_est);
    std::pair<double, double> enhanced_activator(double error, double current_speed, double max_speed, double sensitivity, bool is_cruising_transition);
    double calculate_max_speed_limit(double height);
    double calculate_backward_pitch(double desired_vel_x, double height);


    // --- Core PD Controller Parameters ---
    double Kp_pos_y_, Kd_pos_y_;
    double Kp_yaw_, Kd_yaw_;
    double Kp_z_altitude_;

    // --- Velocity & Angle Limits ---
    double MAX_SPEED_X_;
    double MAX_BACKWARD_SPEED_;
    double MAX_SPEED_Y_;
    double MAX_SPEED_Z_;
    double MAX_SPEED_YAW_;
    double MAX_BACKWARD_PITCH_;
    
    // --- Height-based Speed Limit Parameters ---
    double HEIGHT_SPEED_FACTOR_;
    double MIN_SPEED_LIMIT_;

    // --- Pitch Backward Strategy Parameters ---
    bool USE_PITCH_BACKWARD_;
    double PITCH_TO_VELOCITY_RATIO_;
    double THROTTLE_REDUCTION_RATIO_;
    double BACKWARD_THRESHOLD_;
    double BACKWARD_TRANSITION_START_;

    // --- Distance control switch ---
    bool ENABLE_DISTANCE_CONTROL_;

    // --- Velocity Suppression Parameters ---
    bool VELOCITY_SUPPRESSION_ENABLED_;
    double MAX_SUPPRESSION_FACTOR_;
    double SUPPRESSION_VELOCITY_RANGE_;

    // --- Enhanced Activator (vx control) Parameters ---
    double base_sensitivity_;
    double max_additional_sensitivity_;
    double scaling_factor_;
    double gain_multiplier_;
    double exp_factor_;
    double speed_retention_factor_;
    double TARGET_BBOX_AREA_; // Fetched from state machine config
    double TRANSITION_DURATION_; // Fetched from state machine config


    // --- State Variables ---
    double last_velocity_x_ = 0.0;
    double last_command_x_ = 0.0;
    bool was_cruising_ = false;
    double cruise_entry_speed_ = 0.0;
    ros::Time cruise_entry_time_;

    // --- Debugging ---
    int debug_counter_ = 0;
    int debug_interval_ = 10;
    double adaptive_gain_ = 0.0;
    double max_speed_x_ = 0.0;
};

#endif // CONTROLLER_H
