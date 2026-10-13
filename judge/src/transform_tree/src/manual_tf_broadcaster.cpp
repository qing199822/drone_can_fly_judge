#include "transform_tree/manual_tf_broadcaster.h"

ManualTfBroadcaster::ManualTfBroadcaster(ros::NodeHandle& nh, const std::string& robot_namespace)
    : nh_(nh), robot_namespace_(robot_namespace)
{
    // 构造发布和订阅的话题名称
    std::string tf_topic_name = "/" + robot_namespace_ + "/tf";
    std::string odom_topic_name = "/" + robot_namespace_ + "/global_odom";

    // 初始化发布者
    // 发布到以命名空间为前缀的 /tf 话题，消息类型为 tf2_msgs::TFMessage
    tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>(tf_topic_name, 1);

    // 初始化订阅者
    // 订阅以命名空间为前缀的全局里程计话题
    odom_sub_ = nh_.subscribe(odom_topic_name, 1, &ManualTfBroadcaster::odomCallback, this);

    ROS_INFO("Manual TF Broadcaster for namespace [%s] initialized.", robot_namespace_.c_str());
    ROS_INFO("Subscribing to: %s", odom_topic_name.c_str());
    ROS_INFO("Publishing to: %s", tf_topic_name.c_str());
}

ManualTfBroadcaster::~ManualTfBroadcaster()
{
    // 析构函数，此处无需特殊操作
}

void ManualTfBroadcaster::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 1. 创建一个TransformStamped消息
    geometry_msgs::TransformStamped t;

    // 2. 填充消息头
    // 时间戳直接使用来自 odom 消息的时间戳
    t.header.stamp = msg->header.stamp;
    // 父坐标系固定为 "map"
    t.header.frame_id = "map";
    // 子坐标系固定为 "base_link"
    t.child_frame_id = "base_link";

    // 3. 填充变换数据
    // 平移部分直接复制 odom 消息中的位置
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    // 旋转部分（四元数）直接复制 odom 消息中的姿态
    t.transform.rotation = msg->pose.pose.orientation;

    // 4. 将单个TransformStamped消息封装进TFMessage
    // 因为 /tf 话题的消息类型是 TFMessage，它是一个TransformStamped的数组
    tf2_msgs::TFMessage tf_msg;
    tf_msg.transforms.push_back(t);

    // 5. 发布TFMessage
    tf_pub_.publish(tf_msg);
}
