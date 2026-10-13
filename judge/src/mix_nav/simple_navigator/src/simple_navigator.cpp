#include "simple_navigator/simple_navigator.h"
#include <vector>

// Constructor implementation
SimpleNavigator::SimpleNavigator(ros::NodeHandle& nh, const std::string& vehicle_id)
    : nh_(nh), vehicle_id_(vehicle_id), current_state_(State::IDLE), has_pose_(false),
      current_forward_speed_(0.0) {
    
    // Initialize parameters from the parameter server or use default values
    nh_.param("Kp_linear", Kp_linear_, 0.6);
    nh_.param("Kp_z", Kp_z_, 1.0);
    nh_.param("Kp_yaw", Kp_yaw_, 1.2);
    nh_.param("MAX_SPEED_X", MAX_SPEED_X_, 6.0);
    nh_.param("MAX_ACCEL_X", MAX_ACCEL_X_, 2.0);
    nh_.param("MAX_DECEL_X", MAX_DECEL_X_, 12.0);
    nh_.param("MAX_SPEED_Z", MAX_SPEED_Z_, 1.5);
    nh_.param("MAX_SPEED_YAW", MAX_SPEED_YAW_, 1.5);
    nh_.param("ARRIVAL_TOLERANCE", ARRIVAL_TOLERANCE_, 2.0);
    nh_.param("YAW_ALIGN_THRESHOLD", YAW_ALIGN_THRESHOLD_, M_PI / 6.0); // 30 degrees in radians

    // Setup subscribers and publishers
    pose_sub_ = nh_.subscribe("/" + vehicle_id_ + "/global_pose", 10, &SimpleNavigator::poseCallback, this);
    goal_sub_ = nh_.subscribe("/" + vehicle_id_ + "/move_base_simple/goal", 10, &SimpleNavigator::goalCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/" + vehicle_id_ + "/mux_inputs/navigator/cmd_vel", 10);

    ROS_INFO("[%s_nav] Initialization complete. Waiting for pose...", vehicle_id_.c_str());
}

// Pose callback implementation
void SimpleNavigator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    if (!has_pose_) {
        has_pose_ = true;
        ROS_INFO("[%s_nav] Pose received. Current state: %s", vehicle_id_.c_str(), stateToString(current_state_).c_str());
    }
}

// Goal callback implementation
void SimpleNavigator::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    bool was_navigating = current_state_ == State::NAVIGATING;
    target_goal_ = *msg;
    current_state_ = State::NAVIGATING;
    ROS_INFO_THROTTLE(5.0, "[Navigator] Goal: X=%.2f, Y=%.2f, Z=%.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (!was_navigating) {
        ROS_INFO("[Navigator] State: IDLE -> NAVIGATING");
    }
}

// Convert state enum to string for logging
std::string SimpleNavigator::stateToString(State state) {
    if (state == State::IDLE) return "IDLE";
    return "NAVIGATING";
}

double SimpleNavigator::limitForwardSpeedChange(double desired_speed) {
    const double max_acceleration_step = std::max(0.0, MAX_ACCEL_X_) / 20.0;
    const double max_deceleration_step = std::max(0.0, MAX_DECEL_X_) / 20.0;
    const double speed_error = desired_speed - current_forward_speed_;

    if (speed_error > max_acceleration_step) {
        current_forward_speed_ += max_acceleration_step;
    } else if (speed_error < -max_deceleration_step) {
        current_forward_speed_ -= max_deceleration_step;
    } else {
        current_forward_speed_ = desired_speed;
    }

    return current_forward_speed_;
}

// Main logic loop
void SimpleNavigator::run() {
    ros::Rate rate(20.0);
    geometry_msgs::Twist twist_cmd;

    while (ros::ok()) {
        if (!has_pose_) {
            ROS_INFO_THROTTLE(2, "[%s_nav] Waiting for drone's pose information...", vehicle_id_.c_str());
        } else {
            if (current_state_ == State::IDLE) {
                // Decelerate smoothly before settling into a hover.
                twist_cmd.linear.x = limitForwardSpeedChange(0.0);
                twist_cmd.linear.y = 0;
                twist_cmd.linear.z = 0;
                twist_cmd.angular.x = 0;
                twist_cmd.angular.y = 0;
                twist_cmd.angular.z = 0;
                cmd_vel_pub_.publish(twist_cmd);
                ROS_INFO_THROTTLE(10, "[%s_nav] [IDLE] Waiting for a new waypoint...", vehicle_id_.c_str());

            } else if (current_state_ == State::NAVIGATING) {
                // Calculate position error
                double pos_err_x = target_goal_.pose.position.x - current_pose_.pose.position.x;
                double pos_err_y = target_goal_.pose.position.y - current_pose_.pose.position.y;
                double pos_err_z = target_goal_.pose.position.z - current_pose_.pose.position.z;

                // Check for arrival
                double distance_to_target = std::sqrt(pos_err_x * pos_err_x + pos_err_y * pos_err_y + pos_err_z * pos_err_z);
                if (distance_to_target < ARRIVAL_TOLERANCE_) {
                    ROS_INFO("[%s_nav] Target reached! Distance: %.2fm", vehicle_id_.c_str(), distance_to_target);
                    current_state_ = State::IDLE;
                    ROS_INFO("[%s_nav] State: NAVIGATING -> IDLE", vehicle_id_.c_str());
                    continue;
                }

                // Calculate yaw error
                double desired_yaw = std::atan2(pos_err_y, pos_err_x);
                tf::Quaternion q(
                    current_pose_.pose.orientation.x,
                    current_pose_.pose.orientation.y,
                    current_pose_.pose.orientation.z,
                    current_pose_.pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, current_yaw;
                m.getRPY(roll, pitch, current_yaw);

                double yaw_error = desired_yaw - current_yaw;
                if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
                if (yaw_error < -M_PI) yaw_error += 2 * M_PI;

                // P-controller for velocities
                double desired_vel_x = 0.0, vel_z = 0.0, vel_yaw = 0.0;
                vel_yaw = Kp_yaw_ * yaw_error;

                if (YAW_ALIGN_THRESHOLD_ > 0.0) {
                    double horizontal_distance = std::sqrt(pos_err_x * pos_err_x + pos_err_y * pos_err_y);
                    double yaw_alignment = std::max(0.0, 1.0 - std::abs(yaw_error) / YAW_ALIGN_THRESHOLD_);
                    double aligned_speed = std::max(0.0, std::min(MAX_SPEED_X_, Kp_linear_ * horizontal_distance));
                    desired_vel_x = aligned_speed * yaw_alignment;
                }
                vel_z = Kp_z_ * pos_err_z;
                
                // Apply speed limits
                desired_vel_x = std::max(0.0, std::min(MAX_SPEED_X_, desired_vel_x));
                twist_cmd.linear.x = limitForwardSpeedChange(desired_vel_x);
                twist_cmd.linear.z = std::max(-MAX_SPEED_Z_, std::min(MAX_SPEED_Z_, vel_z));
                twist_cmd.angular.z = std::max(-MAX_SPEED_YAW_, std::min(MAX_SPEED_YAW_, vel_yaw));

                // Set other velocities to zero
                twist_cmd.linear.y = 0;
                twist_cmd.angular.x = 0;
                twist_cmd.angular.y = 0;

                cmd_vel_pub_.publish(twist_cmd);

                ROS_INFO_THROTTLE(1.0, "[%s_nav] [NAVIGATING] -> Target. Dist:%.2fm | Yaw Err:%.1f | Vel(X/Z/Yaw):%.2f/%.2f/%.2f",
                    vehicle_id_.c_str(),
                    distance_to_target,
                    yaw_error * 180.0 / M_PI,
                    twist_cmd.linear.x, twist_cmd.linear.z, twist_cmd.angular.z);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

// Main function
int main(int argc, char **argv) {
    if (argc < 3) {
        ROS_ERROR("Usage: rosrun simple_navigator simple_navigator_node <vehicle_type> <vehicle_id>");
        return 1;
    }
    std::string vehicle_type = argv[1];
    std::string vehicle_id_num = argv[2];
    std::string vehicle_id = vehicle_type + "_" + vehicle_id_num;

    ros::init(argc, argv, "waypoint_navigator_" + vehicle_id);
    ros::NodeHandle nh("~");

    SimpleNavigator navigator(nh, vehicle_id);
    navigator.run();

    return 0;
}
