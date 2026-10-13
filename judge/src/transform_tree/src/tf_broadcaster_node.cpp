#include "transform_tree/manual_tf_broadcaster.h"
#include <ros/ros.h>
#include <string>

int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc < 2)
    {
        ROS_ERROR("Usage: rosrun transform_tree tf_broadcaster_node <robot_namespace>");
        return 1;
    }

    std::string robot_namespace = argv[1];

    // 初始化ROS节点，节点名格式为: <namespace>_manual_tf_broadcaster
    ros::init(argc, argv, robot_namespace + "_manual_tf_broadcaster");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建ManualTfBroadcaster类的实例
    ManualTfBroadcaster broadcaster(nh, robot_namespace);

    // 进入循环，等待回调
    ros::spin();

    return 0;
}
