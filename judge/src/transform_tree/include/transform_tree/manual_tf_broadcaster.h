#ifndef MANUAL_TF_BROADCASTER_H
#define MANUAL_TF_BROADCASTER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <string>

class ManualTfBroadcaster
{
public:
    ManualTfBroadcaster(ros::NodeHandle& nh, const std::string& robot_namespace);
    ~ManualTfBroadcaster();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::NodeHandle nh_; ///< ROS节点句柄
    ros::Subscriber odom_sub_; ///< Odometry话题的订阅者
    ros::Publisher tf_pub_;    ///< /tf话题的发布者
    std::string robot_namespace_; ///< 机器人命名空间
};

#endif
