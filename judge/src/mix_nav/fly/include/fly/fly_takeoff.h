#ifndef FLY_TAKEOFF_H
#define FLY_TAKEOFF_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <topic_tools/MuxSelect.h>
#include <string>
#include <vector>
#include <memory>

namespace fly {

class ConfidentTakeoff {
public:
    /**
     * @brief 构造函数
     * @param drone_name 无人机名称
     * @param drone_quantity 无人机数量
     * @param target_altitude 目标高度
     */
    ConfidentTakeoff(const std::string& drone_name, int drone_quantity, double target_altitude);
    
    /**
     * @brief 运行起飞任务
     */
    void run();

private:
    /**
     * @brief 位姿回调函数
     * @param msg 位姿消息
     * @param drone_id 无人机ID
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int drone_id);
    
    /**
     * @brief 检查所有无人机是否完成任务
     * @return 所有无人机完成任务返回true，否则返回false
     */
    bool allMissionDone() const;

    bool selectMuxInput(int drone_id, const std::string& input_topic);

    // ROS相关
    ros::NodeHandle nh_;
    std::vector<ros::Publisher> cmd_pubs_;
    std::vector<ros::Publisher> vel_pubs_;
    std::vector<ros::Subscriber> pose_subs_;
    std::vector<ros::ServiceClient> mux_select_clients_;
    std::vector<std::string> takeoff_input_topics_;
    std::vector<std::string> navigator_input_topics_;
    
    // 无人机参数
    std::string drone_name_;
    int drone_quantity_;
    double target_altitude_;
    
    // 控制参数
    const double CLIMB_VELOCITY = 0.8;
    const int RATE = 20;
    const double timeout = 15.0;
    const double altitude_tolerance = 0.15;
    
    // 状态变量
    std::vector<geometry_msgs::PoseStamped::ConstPtr> current_poses_;
    std::vector<bool> mission_done_flags_;
    std::vector<bool> pose_received_flags_;
    
    ros::Rate rate_;
};

} // namespace fly

#endif // FLY_TAKEOFF_H
