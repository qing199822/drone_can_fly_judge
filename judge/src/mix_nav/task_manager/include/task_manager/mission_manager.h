// include/task_manager/mission_manager.h

#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <deque>
#include <cmath>
#include <thread>
#include <chrono>
#include <json/json.h>
#include <boost/optional.hpp>

// 定義航點結構
struct Waypoint {
    double x, y, z;
};

class MissionManager {
public:
    // 狀態定義
    enum State {
        STATE_PATROLLING = 0,
        STATE_PAUSED = 1,
        STATE_RESUMING = 2,
        STATE_IDLE = 3
    };

    MissionManager(const std::string& vehicle_id, const std::vector<Waypoint>& waypoints);
    void run_mission();

private:
    // 回調函數
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void control_callback(const std_msgs::String::ConstPtr& msg);

    // 輔助函數
    double get_distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    bool check_stagnation();

    // 節點句柄
    ros::NodeHandle nh_;
    
    // 發布者和訂閱者
    ros::Publisher goal_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber control_sub_;
    ros::Subscriber odom_sub_;

    // 成員變數
    std::string vehicle_id_;
    std::vector<Waypoint> waypoints_;
    geometry_msgs::Pose current_pose_;
    bool has_pose_ = false;

    State state_;
    geometry_msgs::Pose interruption_pose_;
    
    // 雙端隊列存儲返航路徑點
    std::deque<geometry_msgs::PoseStamped> path_stack_;
    ros::Time last_record_time_;
    
    // 配置參數
    double arrival_tolerance_ = 3.0;
    double resume_arrival_tolerance_ = 3.0;
    double record_interval_ = 3.0;
    size_t max_path_points_ = 100;
    bool loop_mission_ = true;
    int current_waypoint_index_ = 0;
    int mission_completed_count_ = 0;

    // 停滯檢測相關變數
    ros::Time stagnation_check_start_time_;
    geometry_msgs::Pose stagnation_previous_pose_;
    bool stagnation_pose_initialized_ = false;
    double stagnation_threshold_ = 0.8;
    double stagnation_timeout_ = 3.0;

    // 用於返航的當前目標點
    boost::optional<geometry_msgs::PoseStamped> current_path_point_ = boost::none;
};

#endif // MISSION_MANAGER_H
