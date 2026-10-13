// src/mission_manager.cpp

#include "task_manager/mission_manager.h"
#include <string>

MissionManager::MissionManager(const std::string& vehicle_id, const std::vector<Waypoint>& waypoints)
    : vehicle_id_(vehicle_id), waypoints_(waypoints), state_(STATE_IDLE) {
    
    ros::NodeHandle private_nh("~");

    // 初始化發布者和訂閱者
    std::string goal_topic = "/" + vehicle_id_ + "/move_base_simple/goal";
    std::string pose_topic = "/" + vehicle_id_ + "/global_pose";
    std::string control_topic = "/" + vehicle_id_ + "/mission/control";
    std::string odom_topic = "/" + vehicle_id_ + "/global_odom";

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    pose_sub_ = nh_.subscribe(pose_topic, 1, &MissionManager::pose_callback, this);
    control_sub_ = nh_.subscribe(control_topic, 1, &MissionManager::control_callback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 1, &MissionManager::odom_callback, this);

    ROS_INFO("[%s] 任務管理器線程已初始化。當前處於待命狀態。", vehicle_id_.c_str());
}

void MissionManager::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = msg->pose;
    if (!has_pose_) {
        has_pose_ = true;
    }
}

void MissionManager::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (state_ == STATE_PAUSED) {
        ros::Time current_time = ros::Time::now();
        if (last_record_time_.is_zero() || (current_time - last_record_time_).toSec() >= record_interval_) {
            geometry_msgs::PoseStamped path_point;
            path_point.header = msg->header;
            path_point.pose = msg->pose.pose;
            
            path_stack_.push_back(path_point);
            last_record_time_ = current_time;

            if (path_stack_.size() > max_path_points_) {
                path_stack_.pop_front();
            }
            ROS_DEBUG("[%s] 記錄路徑點，當前路徑點數量: %zu", vehicle_id_.c_str(), path_stack_.size());
        }
    }
}

void MissionManager::control_callback(const std_msgs::String::ConstPtr& msg) {
    std::string command = msg->data;
    std::transform(command.begin(), command.end(), command.begin(), ::toupper);

    ROS_INFO("[%s] [DEBUG] Control command '%s' received. Current state is %d.", vehicle_id_.c_str(), command.c_str(), state_);

    if (command == "PAUSE" && state_ == STATE_PATROLLING) {
        ROS_INFO("[%s] 收到暫停指令！記錄當前位置為中斷點。", vehicle_id_.c_str());
        interruption_pose_ = current_pose_;
        state_ = STATE_PAUSED;

        path_stack_.clear();
        last_record_time_ = ros::Time(0);
        ROS_INFO("[%s] 開始記錄返航路徑...", vehicle_id_.c_str());

    } else if (command == "RESUME") {
        if (state_ == STATE_PAUSED) {
            ROS_INFO("[%s] 收到恢復指令！開始返回中斷點。", vehicle_id_.c_str());
            ROS_INFO("[%s] 返航路徑包含 %zu 個路徑點", vehicle_id_.c_str(), path_stack_.size());
            state_ = STATE_RESUMING;
            stagnation_pose_initialized_ = false;
        } else if (state_ == STATE_IDLE) {
            ROS_INFO("[%s] 收到首次啟動指令！開始執行巡邏任務。", vehicle_id_.c_str());
            state_ = STATE_PATROLLING;
        }
    } else if (command == "TOGGLE_LOOP") {
        loop_mission_ = !loop_mission_;
        std::string status = loop_mission_ ? "開啟" : "關閉";
        ROS_INFO("[%s] 循環執行模式已%s", vehicle_id_.c_str(), status.c_str());
    }
}

double MissionManager::get_distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

bool MissionManager::check_stagnation() {
    if (!has_pose_ || !stagnation_pose_initialized_) {
        stagnation_previous_pose_ = current_pose_;
        stagnation_check_start_time_ = ros::Time::now();
        stagnation_pose_initialized_ = true;
        return false;
    }

    double distance_moved = get_distance(current_pose_.position, stagnation_previous_pose_.position);

    if (distance_moved > stagnation_threshold_) {
        stagnation_previous_pose_ = current_pose_;
        stagnation_check_start_time_ = ros::Time::now();
        return false;
    }

    if ((ros::Time::now() - stagnation_check_start_time_).toSec() > stagnation_timeout_) {
        ROS_INFO("[%s] 檢測到停滯超過%.1f秒，移動距離: %.2fm", vehicle_id_.c_str(), stagnation_timeout_, distance_moved);
        return true;
    }

    return false;
}

void MissionManager::run_mission() {
    ros::Rate rate(10);

    ROS_INFO("[%s] 等待位姿信息...", vehicle_id_.c_str());
    while (!has_pose_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[%s] 系統就緒，10s後啟動任務。", vehicle_id_.c_str());
    for (int i = 10; i > 0 && ros::ok(); --i) {
        ROS_INFO_THROTTLE(1, "[%s] 倒計時: %d秒", vehicle_id_.c_str(), i);
        ros::Duration(1.0).sleep();
    }

    if (!ros::ok()) return;

    ROS_INFO("[%s] 延遲結束，自動開始巡邏任務。", vehicle_id_.c_str());
    state_ = STATE_PATROLLING;

    while (ros::ok()) {
        ros::spinOnce();

        if (!has_pose_) {
            rate.sleep();
            continue;
        }

        switch (state_) {
            case STATE_IDLE: {
                ROS_INFO_THROTTLE(10, "[%s] [待命中] 等待 'RESUME' 指令...", vehicle_id_.c_str());
                break;
            }
            case STATE_PATROLLING: {
                if (current_waypoint_index_ >= waypoints_.size()) {
                    if (loop_mission_) {
                        mission_completed_count_++;
                        ROS_INFO("[%s] 完成第 %d 輪巡邏任務，重新開始...", vehicle_id_.c_str(), mission_completed_count_);
                        current_waypoint_index_ = 0;
                    } else {
                        ROS_INFO_ONCE("[%s] 所有航點任務已完成！進入懸停狀態。", vehicle_id_.c_str());
                        rate.sleep();
                        continue;
                    }
                }

                const auto& target_point = waypoints_[current_waypoint_index_];
                geometry_msgs::PoseStamped goal_msg;
                goal_msg.header.stamp = ros::Time::now();
                goal_msg.header.frame_id = "map";
                goal_msg.pose.position.x = target_point.x;
                goal_msg.pose.position.y = target_point.y;
                goal_msg.pose.position.z = target_point.z;
                goal_msg.pose.orientation.w = 1.0;

                goal_pub_.publish(goal_msg);

                double distance = get_distance(current_pose_.position, goal_msg.pose.position);
                ROS_INFO_THROTTLE(5, "[%s] [巡邏中] -> 航點 %d。距離: %.2fm", vehicle_id_.c_str(), current_waypoint_index_ + 1, distance);

                if (distance < arrival_tolerance_) {
                    ROS_INFO("[%s] 已到達航點 %d!", vehicle_id_.c_str(), current_waypoint_index_ + 1);
                    current_waypoint_index_++;
                    ros::Duration(1.0).sleep();
                }
                break;
            }
            case STATE_RESUMING: {
                if (!path_stack_.empty()) {
                    if (!current_path_point_) {
                        current_path_point_ = path_stack_.back();
                        path_stack_.pop_back();
                        ROS_INFO("[%s] [返航中] 前往路徑點 %zu，剩餘 %zu 個路徑點", vehicle_id_.c_str(), path_stack_.size() + 1, path_stack_.size());
                    }
                    
                    if (check_stagnation()) {
                        ROS_INFO("[%s] 檢測到停滯，跳過當前路徑點", vehicle_id_.c_str());
                        current_path_point_ = boost::none;
                        stagnation_pose_initialized_ = false;
                        ros::Duration(0.2).sleep();
                        continue;
                    }

                    double distance_to_path_point = get_distance(current_pose_.position, current_path_point_->pose.position);
                    if (distance_to_path_point < 3.0) {
                        ROS_INFO("[%s] 距離路徑點僅 %.2fm，跳過該路徑點", vehicle_id_.c_str(), distance_to_path_point);
                        current_path_point_ = boost::none;
                        stagnation_pose_initialized_ = false;
                        ros::Duration(0.2).sleep();
                        continue;
                    }

                    goal_pub_.publish(*current_path_point_);
                    
                    double distance = get_distance(current_pose_.position, current_path_point_->pose.position);
                    ROS_INFO_THROTTLE(2, "[%s] [返航中] -> 路徑點。距離: %.2fm", vehicle_id_.c_str(), distance);
                    
                    if (distance < resume_arrival_tolerance_) {
                        ROS_INFO("[%s] 已到達路徑點，前往下一個...", vehicle_id_.c_str());
                        current_path_point_ = boost::none;
                        stagnation_pose_initialized_ = false;
                        ros::Duration(0.1).sleep();
                    }

                } else {
                    geometry_msgs::PoseStamped goal_msg;
                    goal_msg.header.stamp = ros::Time::now();
                    goal_msg.header.frame_id = "map";
                    goal_msg.pose = interruption_pose_;
                    
                    goal_pub_.publish(goal_msg);
                    double distance = get_distance(current_pose_.position, interruption_pose_.position);
                    ROS_INFO_THROTTLE(2, "[%s] [返航中] -> 中斷點。距離: %.2fm", vehicle_id_.c_str(), distance);

                    if (distance < resume_arrival_tolerance_) {
                        ROS_INFO("[%s] 已成功返回中斷點！恢復正常巡邏。", vehicle_id_.c_str());
                        current_path_point_ = boost::none;
                        path_stack_.clear();
                        last_record_time_ = ros::Time(0);
                        stagnation_pose_initialized_ = false;
                        state_ = STATE_PATROLLING;
                    }
                }
                break;
            }
            case STATE_PAUSED: {
                geometry_msgs::PoseStamped hover_goal;
                hover_goal.header.stamp = ros::Time::now();
                hover_goal.header.frame_id = "map";
                hover_goal.pose = interruption_pose_;
                goal_pub_.publish(hover_goal);
                ROS_INFO_THROTTLE(5, "[%s] [已暫停] 持續發布懸停指令。", vehicle_id_.c_str());
                break;
            }
        }
        rate.sleep();
    }
}
