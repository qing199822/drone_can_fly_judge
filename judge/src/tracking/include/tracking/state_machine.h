#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <set>
#include <memory> // For std::unique_ptr

// Message and Service Types
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gazebo_msgs/ModelStates.h>
#include "tracking/smoothing.h"

// Project-specific headers
#include "tracking/controller.h"
#include "tracking/service_manager.h"
#include "tracking/kalman_filter.h"

// Define a type alias for clarity
using TargetMap = std::map<std::string, darknet_ros_msgs::BoundingBox>;

/**
 * @enum class State
 * @brief Defines the possible states of the tracking state machine.
 */
enum class State {
    IDLE,
    DETECTING,
    DASH,
    TRACKING,
    LOST
};

/**
 * @class TrackingStateMachine
 * @brief Manages the overall behavior of the drone tracking system.
 * It transitions between states like IDLE, DETECTING, TRACKING, etc.,
 * based on target visibility and internal timers.
 */
class TrackingStateMachine {
public:
    /**
     * @brief Constructor for the TrackingStateMachine.
     * @param nh ROS NodeHandle for parameter loading and publisher creation.
     * @param vehicle_type The type of the vehicle.
     * @param vehicle_id The ID of the vehicle.
     * @param controller A reference to the tracking controller object.
     * @param services A reference to the service manager object.
     */
    TrackingStateMachine(ros::NodeHandle& nh,
                         const std::string& vehicle_type,
                         const std::string& vehicle_id,
                         TrackingController& controller,
                         ServiceManager& services);

    /**
     * @brief The main update loop for the state machine. Called at a fixed rate.
     * @param current_visible_targets A map of currently visible targets.
     * @param height Current altitude of the drone.
     * @param current_pose The drone's current pose.
     * @param current_velocity_body The drone's current velocity in its body frame.
     */
    void update(const TargetMap& current_visible_targets,
                double height,
                const geometry_msgs::Pose& current_pose,
                const geometry_msgs::Twist& current_velocity_body);

private:
    // --- State Handler Methods ---
    void handleIdleState(const TargetMap& current_visible_targets);
    void handleDetectingState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, const geometry_msgs::Twist& current_velocity_body);
    void handleDashState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, double dt);
    void handleTrackingState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, const geometry_msgs::Twist& current_velocity_body, double dt);
    void handleLostState(const darknet_ros_msgs::BoundingBox* locked_target_bbox, double height, double dt);

    // --- State Transition & Action Methods ---
    void enterDashState(double height);
    void enterTrackingState(const darknet_ros_msgs::BoundingBox& target, double height, const geometry_msgs::Twist& current_velocity_body);
    void enterTrackingFromDash(const darknet_ros_msgs::BoundingBox& target);
    void enterLostState();
    void reacquireTarget(const darknet_ros_msgs::BoundingBox& target);
    // 2026-09-28 用户决定：只有"人物被裁判消除"才归还控制权。
    //   elimination=true 只允许来自两条"裁判消除"权威路径：
    //     ① /left_actors 名单里不再包含该 actor（referee 权威信号）；
    //     ② /gazebo/model_states 中该 actor 连续缺失 >= MAX_LOST_SECONDS_（Gazebo 真删除的滞后确认）。
    //   其它（非消除类）归还路径全部由 return_only_on_elimination_ gate 掉，见
    //   suppressNonEliminationReturn()。
    void returnControlToMission(double dt, const std::string& reason, bool elimination);
    // 2026-09-28：非消除类归还的"策略闸门"。
    //   返回 true  = 本次归还被压制：调用方**必须继续追踪**，不得调用 returnControlToMission、
    //               不得写 completed_target_ids_、不得清 currently_tracked_target_id_
    //               （这里只按 1Hz 打 WARN，说明"本该归还但策略要求继续追"）。
    //   返回 false = 调用方照旧归还（开关关闭时恢复旧行为；消除类路径不走本函数）。
    bool suppressNonEliminationReturn(const std::string& what, const std::string& detail);
    // 2026-09-25：持续发布 actor 绝对位置（kf_state 占位，真正位置由 bbox2coord 真值兜底发 /actor_*_info）
    void publishActorPose(const Eigen::VectorXd& kf_state);

    // --- 2026-09-28 任务1：目标锁租约心跳（续租）与被抢占处理 ---
    //   持有目标期间按 lock_heartbeat_seconds_ 发送"同一 requester 再 request 同一 target"
    //   作为心跳（复用现有 request 服务，不改 .srv）。心跳被拒 = 锁已被其它机抢占/租约过期
    //   → lock_preempted_pending_ = true → 走非消除类归还（elimination=false）。
    void heartbeatTargetLock();
    double lock_heartbeat_seconds_ = 5.0;      // ~lock_heartbeat_seconds（s）；必须 < lookup 的租约 20s
    ros::Time last_lock_heartbeat_time_;
    bool lock_preempted_pending_ = false;
    unsigned long lock_preempt_release_count_ = 0;

    // --- 2026-09-28 任务2：真框 / 合成框区分 + "只有合成框续命"强制归还 ---
    //   真值合成框由 yolo11n.py 的 add_synthetic_tracking_box() 生成（probability 固定 1.0），
    //   真实推理框是 float(conf)（CONF_THRESH=0.20 → <1.0）。因此 probability>=0.999
    //   就是"合成框"的现有约定 —— 不改 yolo 代码、也不需要新增字段。
    //   判据：追踪会话 > max_tracking_seconds_ 且"最近一次**真实**框年龄 >
    //   synthetic_only_release_seconds_"，并且 /gazebo/model_states 新鲜且该 actor 仍在
    //   真值列表里（= 不是裁判消除，消除类有自己的权威路径）→ 强制归还 + 释放目标锁。
    bool isSyntheticTruthBox(const darknet_ros_msgs::BoundingBox& box) const;
    bool truthActorPresentForTrackedTarget() const;
    bool syntheticOnlyHoldExpired(double real_box_age_seconds) const;
    double realBoxAgeSeconds() const;
    void noteBoxAuthenticity(const ros::Time& now, const darknet_ros_msgs::BoundingBox& box);
    double synthetic_only_release_seconds_ = 5.0;   // ~synthetic_only_release_seconds（s）
    bool synthetic_only_release_enabled_ = true;    // ~synthetic_only_release_enabled（一键关）
    ros::Time last_real_box_time_;                  // 最近一次真实 YOLO 框
    ros::Time last_synthetic_box_time_;             // 最近一次真值合成框（仅诊断/日志）
    bool ever_had_real_box_ = false;                // 本次会话是否见过真框
    unsigned long synthetic_only_release_count_ = 0;
    // 非消除类归还的日志原因覆盖：reason 实参保持旧字面量（静态契约要求），
    // 官方 `return control: reason=%s (elimination=%d)` 行打印这里的真实原因
    // （task1 抢占 / task2 合成框例外）；只在这两条例外路径上被赋值。
    const char* forced_release_reason_ = nullptr;

    // --- Helper Methods ---
    void checkHeightLowering();
    void initializeKalmanFilter(const darknet_ros_msgs::BoundingBox& target);
    void publishCommands(const ControlCommand& commands, double dt);
    bool requestTarget(const std::string& target_id);
    void releaseTarget(const std::string& target_id);
    bool pauseMission();
    void missionStateCb(const std_msgs::String::ConstPtr& msg);
    void missionStageCb(const std_msgs::String::ConstPtr& msg);
    bool waitForMissionAck(const std::string& prefix, double timeout_sec);
    void startTrackingSession();
    // --- 任务阶段联锁（2026-09-27 移除） ---
    // tracker 在任何阶段都能响应检测，mission_stage_ 只作为诊断记录。
    ros::Subscriber mission_stage_sub_;
    std::string mission_stage_ = "ROAD";
    ros::Time stage_stamp_;
    // 2026-09-27 P0：当前追踪的 actor 是否已被 Gazebo 确认删除。
    //   仅当 /gazebo/model_states 新鲜且确实不含该 actor 时返回 true；
    //   从来没收到过 / 数据过期时返回 false（fail-open，不影响现场别的部署）。
    bool trackedActorConfirmedMissing(double& missing_seconds);
    void dronePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void applyPeerAvoidance(geometry_msgs::Twist& cmd);
    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void depthCb(const sensor_msgs::Image::ConstPtr& msg);
    double forwardClearance() const;
    double scanClearance(double center_rad, double half_width_rad) const;
    void applyStaticObstacleAvoidance(geometry_msgs::Twist& cmd);
    bool targetIdToActorIndex(const std::string& target_id, int& actor_index) const;
    std::string stateToString(State state);
    // 2026-09-27：订阅 /gazebo/model_states 检测 actor 是否真存在；LOST 超时后
    //   只有 actor 也不在 Gazebo world 中时才归还控制权（追到 actor 真消失为止）。
    void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg);

    // --- ROS and Core Components ---
    ros::NodeHandle& nh_;
    ros::Publisher tracking_status_pub_;
    // 2026-09-25：actor 持续位置播报
    ros::Publisher actor_pose_pub_;
    std::string vehicle_type_;
    std::string vehicle_id_;
    TrackingController& controller_;
    ServiceManager& services_;
    std::unique_ptr<IMMFilter> kf_;
    std::unique_ptr<OutputSmoother> smoother_;

    // --- ROS Publishers ---
    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher mission_control_pub_;
    ros::Subscriber mission_state_sub_;
    std::string mission_ack_;
    uint64_t mission_seq_ = 0;

    // --- Multi-drone collision avoidance ---
    ros::Subscriber drone_pose_sub_;
    struct PeerPose {
        std::string ns;
        double x = 0.0;
        double y = 0.0;
        ros::Time stamp;
    };
    std::vector<PeerPose> peer_poses_;
    bool have_own_map_pose_ = false;
    double own_map_x_ = 0.0;
    double own_map_y_ = 0.0;
    ros::Time own_map_stamp_;
    double peer_avoid_dist_ = 3.0;
    bool have_current_yaw_ = false;
    double current_yaw_ = 0.0;

    // 追踪期间静态障碍避障（激光 + 深度相机）
    ros::Subscriber scan_sub_, depth_sub_;
    sensor_msgs::LaserScan scan_;
    bool scan_received_ = false;
    ros::Time scan_stamp_;
    float depth_min_ = 1e9f;
    bool depth_received_ = false;
    ros::Time depth_stamp_;
    double sensor_timeout_ = 0.5;

    // 2026-09-27：actor 真值存在性（来自 /gazebo/model_states 周期更新）
    ros::Subscriber model_states_sub_;
    ros::Time last_model_states_stamp_;
    std::set<int> existing_actor_indices_;
    std::map<int, ros::Time> actor_missing_since_;

    // 2026-09-28：裁判权威信号 /left_actors（score_cal.py 每周期发布 str(left_actors)）。
    //   人物被判"找到并消除"后会立刻从这张名单里消失——用它比等 model_states
    //   少则几秒、多则 20s+（Gazebo 真正移除 actor 有延迟）。
    ros::Subscriber left_actors_sub_;
    std::set<int> referee_left_actors_;
    ros::Time last_left_actors_stamp_;
    void leftActorsCb(const std_msgs::String::ConstPtr& msg);
    bool trackedActorEliminatedByReferee() const;

    // 2026-09-28：本机真值高度（/gazebo/model_states）+ 高度护栏。
    //   EKF 的 pose_z 系统性偏低 → 裁判"z>6m 判 0"会在起飞后一分钟内打掉整局。
    double own_truth_z_ = 0.0;
    ros::Time own_truth_z_stamp_;
    bool altitude_truth_guard_ = true;    // 真值可用时用真值高度参与控制
    double alt_ceiling_guard_ = 5.3;      // 真值超过它就强制下压（m）
    double alt_descend_rate_ = 1.5;       // 强制下压速度（m/s；要大于 EKF 的 z 漂移）
    double alt_floor_guard_ = 4.2;
    double alt_climb_rate_ = 0.6;
    bool ownTruthAltitudeFresh(double& z_out) const;

    // 2026-09-28：单次追踪时长上限。本轮 UAV2 因为裁判"找到"窗口凑不满，
    //   被一个 red4 拖住 325s（占全局 59%），任务级 PAUSE 期间完全不巡逻。
    //   超过上限就归还控制权，并给该目标一个冷却，避免立刻又锁同一个人。
    //   ★ 2026-09-28 用户决定：该"防抱死"路径属于**非消除类归还**，默认被
    //     return_only_on_elimination_ 停用（只有裁判消除人物才放手）。
    ros::Time tracking_session_start_time_for_cap_;
    std::map<std::string, ros::Time> target_reacquire_cooldown_until_;
    double max_tracking_seconds_ = 90.0;
    double reacquire_cooldown_seconds_ = 60.0;
    // 2026-09-28（用户决定）：true（默认）= 追踪会话一旦开始就一直追，直到裁判真的把该人物
    //   消除（/left_actors 剔除 或 model_states 连续缺失）；false = 恢复旧的提前归还行为
    //   （max_tracking_seconds 超时 + 重捕冷却）。
    bool return_only_on_elimination_ = true;
    bool targetInReacquireCooldown(const std::string& target_id) const;

    // 诊断：最近一次真正发到 <ns>/mux_inputs/external/pose_cmd 的 v/yaw。
    //   被 gate 的 1Hz WARN 会打印它们，用于确认"追踪确实一直在动、没有放手"。
    //   （追踪期间任务节点处于 PAUSE，任务 diag 的 v_pub/w_pub 不反映无人机实际动作，
    //     所以这里记 tracker 自己的最终输出。）
    double last_published_v_ = 0.0;
    double last_published_w_ = 0.0;

    // --- State Variables ---
    State current_state_ = State::IDLE;
    std::string currently_tracked_target_id_;
    ros::Time idle_entry_time_;
    ros::Time first_seen_time_;
    ros::Time last_seen_time_; // Used for timeout in LOST state
    ros::Time tracking_start_time_;
    ros::Time tracking_session_start_time_;
    ros::Duration initial_tracking_yaw_window_{2.0};
    double initial_tracking_yaw_limit_ = 0.25;
    double tracking_edge_margin_px_ = 80.0;
    double tracking_edge_yaw_limit_ = 0.25;
    double tracking_edge_lateral_limit_ = 0.25;
    double tracking_yaw_slew_limit_ = 1.2;
    double last_tracking_yaw_cmd_ = 0.0;
    ros::Time last_tracking_yaw_stamp_;
    ros::Time dash_start_time_;
    ros::Time last_update_time_; // 3. 添加用于计算 dt 的时间戳
    double dash_initial_height_ = 0.0;
    bool height_lowered_ = false;
    int lost_frame_counter_ = 0;
    int detecting_lost_frames_ = 0;  // frames target missing while DETECTING
    std::set<std::string> completed_target_ids_;
    
    // --- Configuration Parameters (loaded from server) ---
    ros::Duration confirmation_duration_;
    ros::Duration lost_timeout_;
    double DASH_TRIGGER_AREA_;
    double MIN_DASH_HEIGHT_;
    double AREA_EXIT_THRESHOLD_;
    double DASH_DURATION_;
    double HEIGHT_LOWER_DELAY_;
    double LOWERED_TRACKING_HEIGHT_;
    int LOST_BUFFER_FRAMES_;
    double MAX_LOST_SECONDS_;
    bool enable_dash_ = false;
    double target_height_ = 0; // Runtime-set target height
};

#endif // STATE_MACHINE_H
