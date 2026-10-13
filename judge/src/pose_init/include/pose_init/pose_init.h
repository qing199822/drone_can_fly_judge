#ifndef POSE_INIT_H
#define POSE_INIT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

// 定义一个结构体来存储坐标偏移量
struct DroneOffset {
    double x;
    double y;
    double z;
};

/**
 * @class PoseTransformer
 * @brief 此类用于为单个无人机转换和转发其位姿信息。
 * 
 * 它订阅一个局部坐标系的位姿和里程计话题，加上一个固定的偏移量，
 * 然后将结果发布到一个新的话题上，作为其在全局坐标系中的位姿。
 */
class PoseTransformer {
public:
    /**
     * @brief PoseTransformer 类的构造函数。
     * @param nh ROS 节点句柄。
     * @param namespace_str 无人机的命名空间, 例如 "typhoon_h480_0"。
     * @param offset 该无人机在地图坐标系中的起始偏移量。
     */
    PoseTransformer(ros::NodeHandle& nh, const std::string& namespace_str, const DroneOffset& offset);

    /**
     * @brief 析构函数。
     */
    ~PoseTransformer();

private:
    /**
     * @brief 接收到局部 PoseStamped 消息后的回调函数。
     * @param msg 接收到的局部 PoseStamped 消息的常量指针。
     */
    void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief 接收到局部 Odometry 消息后的回调函数。
     * @param msg 接收到的局部 Odometry 消息的常量指针。
     */
    void localOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // ROS 相关的句柄、发布者和订阅者
    ros::NodeHandle nh_;
    ros::Publisher global_pose_pub_;
    ros::Publisher global_odom_pub_;
    ros::Subscriber local_pose_sub_;
    ros::Subscriber local_odom_sub_;

    // 无人机命名空间和偏移量
    std::string namespace_;
    DroneOffset offset_;
};

#endif // POSE_INIT_H
