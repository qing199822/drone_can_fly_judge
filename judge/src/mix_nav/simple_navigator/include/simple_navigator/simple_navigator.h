#ifndef SIMPLE_NAVIGATOR_H
#define SIMPLE_NAVIGATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h> // <-- 正确的头文件#include <string>
#include <cmath>
#include <algorithm>

// Define states as a simple enumeration for type safety and clarity
enum class State {
    IDLE,
    NAVIGATING
};

class SimpleNavigator {
public:
    // Constructor: Initializes the node, sets up subscribers and publishers
    SimpleNavigator(ros::NodeHandle& nh, const std::string& vehicle_id);

    // Main loop for the navigator's logic
    void run();

private:
    // Callback functions for updating drone's current pose and receiving new goals
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // Helper function to get a string representation of the current state
    std::string stateToString(State state);
    double limitForwardSpeedChange(double desired_speed);

    // ROS specific members
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_vel_pub_;

    // Global variables from the Python script are now private members of the class
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped target_goal_;
    State current_state_;
    bool has_pose_ = false; // Flag to check if we have received pose information

    // Controller parameters
    double Kp_linear_;
    double Kp_z_;
    double Kp_yaw_;

    // Speed limits
    double MAX_SPEED_X_;
    double MAX_ACCEL_X_;
    double MAX_DECEL_X_;
    double MAX_SPEED_Z_;
    double MAX_SPEED_YAW_;
    double current_forward_speed_;

    // Other parameters
    double ARRIVAL_TOLERANCE_;
    double YAW_ALIGN_THRESHOLD_;

    std::string vehicle_id_;
};

#endif // SIMPLE_NAVIGATOR_H
