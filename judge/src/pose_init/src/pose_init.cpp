#include "pose_init/pose_init.h"
#include <map>
#include <memory> // 用于 std::unique_ptr
#include <vector>

/**
 * 初始化一个无人机的位姿转换器。
 */
PoseTransformer::PoseTransformer(ros::NodeHandle& nh, const std::string& namespace_str, const DroneOffset& offset)
    : nh_(nh), namespace_(namespace_str), offset_(offset) {
    
    // 输入话题：无人机相对于其起飞点的局部位置
    std::string input_pose_topic = "/" + namespace_ + "/mavros/local_position/pose";
    std::string input_odom_topic = "/" + namespace_ + "/mavros/local_position/odom";

    // 输出话题：无人机在全局地图坐标系中的位置
    std::string output_pose_topic = "/" + namespace_ + "/global_pose";
    std::string output_odom_topic = "/" + namespace_ + "/global_odom";

    // 创建发布者
    global_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_pose_topic, 10);
    global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic, 10);

    // 创建订阅者
    local_pose_sub_ = nh_.subscribe(input_pose_topic, 10, &PoseTransformer::localPoseCallback, this);
    local_odom_sub_ = nh_.subscribe(input_odom_topic, 10, &PoseTransformer::localOdomCallback, this);

    ROS_INFO("为 %s 初始化坐标转换器。", namespace_.c_str());
    ROS_INFO("  -> 订阅: %s 和 %s", input_pose_topic.c_str(), input_odom_topic.c_str());
    ROS_INFO("  -> 发布: %s 和 %s", output_pose_topic.c_str(), output_odom_topic.c_str());
}

PoseTransformer::~PoseTransformer() {}

/**
 * 接收到局部PoseStamped消息后的回调函数。
 */
void PoseTransformer::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& local_pose_msg) {
    // 创建一个新的PoseStamped消息用于发布
    geometry_msgs::PoseStamped global_pose_msg;

    // 1. 复制header和orientation信息，保持不变
    global_pose_msg.header = local_pose_msg->header;
    global_pose_msg.pose.orientation = local_pose_msg->pose.orientation;

    // 2. 加上偏移量，计算全局坐标
    global_pose_msg.pose.position.x = local_pose_msg->pose.position.x + offset_.x;
    global_pose_msg.pose.position.y = local_pose_msg->pose.position.y + offset_.y;
    global_pose_msg.pose.position.z = local_pose_msg->pose.position.z + offset_.z;

    // 3. 发布转换后的消息
    global_pose_pub_.publish(global_pose_msg);
}

/**
 * 接收到局部Odometry消息后的回调函数。
 */
void PoseTransformer::localOdomCallback(const nav_msgs::Odometry::ConstPtr& local_odom_msg) {
    // 创建一个新的Odometry消息用于发布
    nav_msgs::Odometry global_odom_msg;

    // 1. 复制header和child_frame_id信息
    global_odom_msg.header = local_odom_msg->header;
    global_odom_msg.child_frame_id = local_odom_msg->child_frame_id;

    // 2. 复制姿态、协方差和速度信息（保持不变）
    global_odom_msg.pose.pose.orientation = local_odom_msg->pose.pose.orientation;
    global_odom_msg.pose.covariance = local_odom_msg->pose.covariance;
    global_odom_msg.twist = local_odom_msg->twist;

    // 3. 加上偏移量，计算全局坐标
    global_odom_msg.pose.pose.position.x = local_odom_msg->pose.pose.position.x + offset_.x;
    global_odom_msg.pose.pose.position.y = local_odom_msg->pose.pose.position.y + offset_.y;
    global_odom_msg.pose.pose.position.z = local_odom_msg->pose.pose.position.z + offset_.z;
    
    // 4. 发布转换后的消息
    global_odom_pub_.publish(global_odom_msg);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pose_transformer_node");
    ros::NodeHandle nh;

    // ===== 出生点布局（环境适配层） =====
    // 这里把「无人机出生点 = mavros 局部 odom 原点在世界系中的位置」参数化。
    //   世界坐标 = 局部 odom 坐标 + offset
    // 默认值 = 本地调试布局：
    //   x = -17 + 3 * (id % 2)
    //   y = -3  + 3 * (id // 2)
    // 比赛时组委会用官方 XTDrone sitl_config/launch/robocup.launch，出生点是
    //   x =  0 + 3 * (id % 2)
    //   y = -3 + 3 * (id // 2)
    // 只差一个 spawn_origin_x，所以换环境只需覆盖参数，不必改代码：
    //   rosparam set /multi_drone_pose_transformer/spawn_origin_x 0
    // 也可用 <ns>/offset_x / <ns>/offset_y 逐机覆盖（见下）。
    ros::NodeHandle pnh("~");
    double spawn_origin_x = -17.0, spawn_origin_y = -3.0;
    double spawn_stride_x = 3.0, spawn_stride_y = 3.0;
    int spawn_num_drones = 6;
    std::string spawn_vehicle_type = "typhoon_h480";
    pnh.param<double>("spawn_origin_x", spawn_origin_x, spawn_origin_x);
    pnh.param<double>("spawn_origin_y", spawn_origin_y, spawn_origin_y);
    pnh.param<double>("spawn_stride_x", spawn_stride_x, spawn_stride_x);
    pnh.param<double>("spawn_stride_y", spawn_stride_y, spawn_stride_y);
    pnh.param<int>("num_drones", spawn_num_drones, spawn_num_drones);
    pnh.param<std::string>("vehicle_type", spawn_vehicle_type, spawn_vehicle_type);

    std::map<std::string, DroneOffset> drone_offsets;
    for (int i = 0; i < spawn_num_drones; ++i) {
        const std::string name = spawn_vehicle_type + "_" + std::to_string(i);
        // 与 spawn_drones.launch / serial_swarm_bringup / area_mission_node /
        // scan_map_fuser 使用同一公式，保证六处一致。
        drone_offsets[name] = {spawn_origin_x + spawn_stride_x * (i % 2),
                               spawn_origin_y + spawn_stride_y * (i / 2),
                               0.0};
    }

    // The input MAVROS pose is local odom. Keep the published vision pose
    // explicitly world aligned; offsets can be overridden per vehicle when a
    // different map or spawn layout is used.
    for (auto& pair : drone_offsets) {
        const std::string key = pair.first;
        pnh.param<double>(key + "/offset_x", pair.second.x, pair.second.x);
        pnh.param<double>(key + "/offset_y", pair.second.y, pair.second.y);
        pnh.param<double>(key + "/offset_z", pair.second.z, pair.second.z);
    }

    // 为字典中的每一架无人机创建一个转换器实例
    std::vector<std::unique_ptr<PoseTransformer>> transformers;
    for (const auto& pair : drone_offsets) {
        transformers.push_back(std::make_unique<PoseTransformer>(nh, pair.first, pair.second));
    }

    ROS_INFO("所有无人机坐标转换器已启动。节点将持续运行...");

    // 保持节点运行，直到被关闭
    ros::spin();

    return 0;
}
