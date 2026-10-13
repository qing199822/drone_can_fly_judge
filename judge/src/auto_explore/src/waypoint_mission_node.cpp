// waypoint_mission_node.cpp — 直飞航点任务节点（保留 explore_node 不动）。
//
// 与 explore_node 的区别：这里不做 A* 全局规划、不做 frontier 探索，
// 航点之间直接直线飞行，局部避障交给 MPPI（mppi_planner.hpp）。
//
// 坐标系约定：waypoints 使用世界/map 坐标；MAVROS pose 和控制目标使用本机 odom，
// 仅在发送目标和发布 /drone_poses 时做一次 spawn 转换。
//
// 任务流程：
//   TAKEOFF  -> 爬升到 target_altitude
//   MISSION  -> 按 waypoints 顺序直飞（MPPI 避障），到达 wp_reach_tolerance 后切下一个
//   tracking 接管时：收到 PAUSE 后记录当前 odom 位置并悬停
//   收到 RESUME 后：先飞回暂停点，再继续原来的航点索引
//
// 话题/服务复用 explore_node 的接口：
//   输出 cmd_vel 到 /<vehicle>/mux_inputs/navigator/cmd_vel
//   XTDrone 命令 /xtdrone/<vehicle>/cmd
//   订阅 /<vehicle>/mission/control  (PAUSE / RESUME)
//   订阅 /<vehicle>/scan_filtered
//   订阅 /<vehicle>/mavros/local_position/pose
//   发布 /drone_poses（map 坐标，frame_id=vehicle）

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <XmlRpcValue.h>

#include "auto_explore/mppi_planner.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace
{
double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
double normAngle(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}
}

class WaypointMissionNode
{
public:
  WaypointMissionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
  {
    std::string vehicle;
    pnh_.param<std::string>("vehicle", vehicle, "typhoon_h480_0");
    vehicle_ = vehicle;

    // 从车辆名解析 id，并复算 6 机 spawn 偏移（与 scan_map_fuser 一致），
    // 用于把 odom 坐标换算成 map 坐标发布到 /drone_poses。
    const size_t pos = vehicle_.find_last_of('_');
    if (pos != std::string::npos)
      drone_id_ = std::atoi(vehicle_.c_str() + pos + 1);
    spawn_x_ = -17.0 + 3.0 * (drone_id_ % 2);
    spawn_y_ = -3.0 + 3.0 * (drone_id_ / 2);

    // 话题
    pnh_.param<std::string>("pose_topic", pose_topic_,
                            "/" + vehicle_ + "/mavros/local_position/pose");
    pnh_.param<std::string>("scan_topic", scan_topic_,
                            "/" + vehicle_ + "/scan_filtered");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_,
                            "/" + vehicle_ + "/mux_inputs/navigator/cmd_vel");
    pnh_.param<std::string>("cmd_topic", cmd_topic_,
                            "/xtdrone/" + vehicle_ + "/cmd");

    // 飞行参数
    pnh_.param<double>("target_altitude", target_altitude_, 3.0);
    pnh_.param<double>("max_z", max_z_, 0.8);
    pnh_.param<double>("k_z", k_z_, 0.6);
    pnh_.param<double>("takeoff_pos_kp", takeoff_pos_kp_, 0.5);
    pnh_.param<double>("takeoff_max_horiz", takeoff_max_horiz_, 1.0);
    pnh_.param<double>("control_rate", control_rate_, 20.0);
    pnh_.param<double>("wp_reach_tolerance", wp_reach_tolerance_, 2.5);
    pnh_.param<double>("smooth_alpha", smooth_alpha_, 0.3);
    pnh_.param<double>("avoid_dist", avoid_dist_, 10.0);

    // MPPI 参数（与 explore_node 保持同名，便于 launch 复用）
    mppi::Params mppi_prm;
    pnh_.param<int>("mppi_num_samples", mppi_prm.num_samples, 300);
    pnh_.param<int>("mppi_horizon", mppi_prm.horizon, 20);
    pnh_.param<double>("mppi_dt", mppi_prm.dt, 0.10);
    pnh_.param<double>("mppi_max_v", mppi_prm.max_v, 5.0);
    pnh_.param<double>("mppi_min_v", mppi_prm.min_v, 0.0);
    pnh_.param<double>("mppi_max_w", mppi_prm.max_w, 1.2);
    pnh_.param<double>("mppi_sigma_v", mppi_prm.sigma_v, 0.8);
    pnh_.param<double>("mppi_sigma_w", mppi_prm.sigma_w, 0.6);
    pnh_.param<double>("mppi_lambda", mppi_prm.lambda, 0.15);
    pnh_.param<double>("mppi_v_des", mppi_prm.v_des, 4.0);
    pnh_.param<double>("mppi_goal_weight", mppi_prm.goal_weight, 1.2);
    pnh_.param<double>("mppi_obstacle_weight", mppi_prm.obstacle_weight, 40.0);
    pnh_.param<double>("mppi_yaw_weight", mppi_prm.yaw_weight, 0.05);
    pnh_.param<double>("mppi_speed_weight", mppi_prm.speed_weight, 0.4);
    pnh_.param<double>("mppi_field_radius", mppi_prm.field_radius, 18.0);
    pnh_.param<double>("mppi_field_res", mppi_prm.field_res, 0.15);
    pnh_.param<double>("mppi_safe_radius", mppi_prm.safe_radius, 2.5);
    mppi_.configure(mppi_prm);

    // 航点使用世界/map 坐标，发送控制目标时转换为 odom。
    XmlRpc::XmlRpcValue wps;
    if (pnh_.getParam("waypoints", wps) &&
        wps.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < wps.size(); ++i)
      {
        const double x = static_cast<double>(wps[i][0]);
        const double y = static_cast<double>(wps[i][1]);
        waypoints_.push_back(std::make_pair(x, y));
      }
    }
    ROS_INFO("[waypoint_mission] %s: %zu waypoints, spawn map offset (%.1f, %.1f)",
             vehicle_.c_str(), waypoints_.size(), spawn_x_, spawn_y_);

    // Subscribers
    pose_sub_ = nh_.subscribe(pose_topic_, 1, &WaypointMissionNode::poseCb, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &WaypointMissionNode::scanCb, this);
    mission_control_sub_ = nh_.subscribe("/" + vehicle_ + "/mission/control", 1,
                                         &WaypointMissionNode::missionControlCb, this);
    drone_pose_sub_ = nh_.subscribe("/drone_poses", 10,
                                    &WaypointMissionNode::dronePoseCb, this);

    // Publishers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    cmd_pub_ = nh_.advertise<std_msgs::String>(cmd_topic_, 1);
    drone_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/drone_poses", 1);

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                     &WaypointMissionNode::controlCb, this);
  }

private:
  enum State { TAKEOFF, MISSION, DONE };

  // ------------------------------ callbacks ------------------------------
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    pose_z_ = msg->pose.position.z;
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                      msg->pose.orientation.z, msg->pose.orientation.w);
    double r, p;
    tf2::Matrix3x3(q).getRPY(r, p, pose_yaw_);
    pose_received_ = true;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    scan_ = *msg;
    scan_received_ = true;
  }

  void missionControlCb(const std_msgs::String::ConstPtr& msg)
  {
    if (msg->data == "PAUSE")
    {
      pause_requested_ = true;
      resume_requested_ = false;
    }
    else if (msg->data == "RESUME")
    {
      resume_requested_ = true;
      pause_requested_ = false;
    }
  }

  void dronePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (msg->header.frame_id.empty() || msg->header.frame_id == vehicle_)
      return;
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    if (age > 1.0) return;
    bool updated = false;
    for (PeerPose& pp : peer_poses_)
    {
      if (pp.ns == msg->header.frame_id)
      {
        pp.x = msg->pose.position.x;
        pp.y = msg->pose.position.y;
        pp.stamp = msg->header.stamp;
        updated = true;
        break;
      }
    }
    if (!updated)
      peer_poses_.push_back({msg->header.frame_id, msg->pose.position.x,
                             msg->pose.position.y, msg->header.stamp});
  }

  void sendCmd(const std::string& cmd)
  {
    std_msgs::String m;
    m.data = cmd;
    cmd_pub_.publish(m);
  }

  // ------------------------------ helpers ------------------------------
  void publishOwnPose()
  {
    const ros::Time now = ros::Time::now();
    if ((now - last_pose_pub_).toSec() < 0.2) return;
    last_pose_pub_ = now;
    geometry_msgs::PoseStamped p;
    p.header.stamp = now;
    p.header.frame_id = vehicle_;
    p.pose.position.x = pose_x_ + spawn_x_;
    p.pose.position.y = pose_y_ + spawn_y_;
    p.pose.position.z = pose_z_;
    p.pose.orientation.w = 1.0;
    drone_pose_pub_.publish(p);
  }

  // 机头正前方 30° 扇区内的最小激光 clearance（m）。
  // 用于 MPPI 输出后的最后硬刹车：MPPI 只是采样优化，必须有一层
  // 确定性的距离保护，保证任何情况下都不会撞墙。
  double forwardClearance()
  {
    if (!scan_received_) return 1e9;
    const int n = static_cast<int>(scan_.ranges.size());
    if (n == 0) return 1e9;
    const double half_cone = 15.0 * M_PI / 180.0;
    double m = 1e9;
    for (int i = 0; i < n; ++i)
    {
      const double r = scan_.ranges[i];
      if (!(r >= scan_.range_min && r <= scan_.range_max) || !std::isfinite(r))
        continue;
      const double a = scan_.angle_min + i * scan_.angle_increment;
      if (std::fabs(a) <= half_cone)
        m = std::min(m, static_cast<double>(r));
    }
    return m;
  }

  // 机体系指定角度方向 ±15° 锥内的最小激光 clearance（m）。
  // 用于防止 MPPI/机间避障把飞机转向一个有障碍的方向。
  double sideClearance(double dir_rad)
  {
    if (!scan_received_) return 1e9;
    const int n = static_cast<int>(scan_.ranges.size());
    if (n == 0) return 1e9;
    const double half_cone = 15.0 * M_PI / 180.0;
    double m = 1e9;
    for (int i = 0; i < n; ++i)
    {
      const double r = scan_.ranges[i];
      if (!(r >= scan_.range_min && r <= scan_.range_max) || !std::isfinite(r))
        continue;
      const double a = scan_.angle_min + i * scan_.angle_increment;
      if (std::fabs(normAngle(a - dir_rad)) <= half_cone)
        m = std::min(m, static_cast<double>(r));
    }
    return m;
  }

  // 简单机间避障：与 explore_node 相同的策略，在 spawn 附近 12m 内不启用。
  void avoidPeers(geometry_msgs::Twist& cmd)
  {
    if (!pose_received_) return;
    const double mx = pose_x_ + spawn_x_;
    const double my = pose_y_ + spawn_y_;
    if (std::hypot(mx - spawn_x_, my - spawn_y_) < 12.0) return;

    double best_d2 = avoid_dist_ * avoid_dist_;
    double bx = 0.0, by = 0.0;
    const ros::Time now = ros::Time::now();
    for (const PeerPose& pp : peer_poses_)
    {
      if ((now - pp.stamp).toSec() > 0.5) continue;
      const double ddx = pp.x - mx, ddy = pp.y - my;
      const double d2 = ddx * ddx + ddy * ddy;
      if (d2 < best_d2) { best_d2 = d2; bx = ddx; by = ddy; }
    }
    if (best_d2 >= avoid_dist_ * avoid_dist_) return;

    const double d = std::sqrt(best_d2);
    const double escape = std::atan2(-by, -bx);
    const double yaw_err = normAngle(escape - pose_yaw_);
    cmd.angular.z = clamp(0.8 * yaw_err * 3.0, -0.8, 0.8);  // 限制幅度，避免猛转出道路撞墙
    const double closeness = 1.0 - d / avoid_dist_;
    cmd.linear.x *= (1.0 - 0.9 * closeness);
    if (d < 1.5) cmd.linear.x = 0.0;
  }

  void publishSmooth(geometry_msgs::Twist& cmd)
  {
    const ros::Time now = ros::Time::now();
    double dt = last_smooth_time_.is_zero() ? (1.0 / control_rate_)
                                            : (now - last_smooth_time_).toSec();
    last_smooth_time_ = now;
    dt = std::max(0.01, std::min(dt, 0.5));

    smooth_x_ += smooth_alpha_ * (cmd.linear.x - smooth_x_);
    smooth_yaw_ += smooth_alpha_ * (cmd.angular.z - smooth_yaw_);
    cmd.linear.x = smooth_x_;
    cmd.angular.z = smooth_yaw_;
    cmd_vel_pub_.publish(cmd);
  }

  // ------------------------------ control ------------------------------
  void controlCb(const ros::TimerEvent&)
  {
    geometry_msgs::Twist cmd;

    if (pose_received_) publishOwnPose();

    // 处理 tracking 的 PAUSE/RESUME 请求
    if (pause_requested_ && !paused_)
    {
      pause_x_ = pose_x_;
      pause_y_ = pose_y_;
      resume_wp_idx_ = wp_idx_;
      paused_ = true;
      return_to_pause_ = false;
      ROS_INFO("[waypoint_mission] %s PAUSED at odom(%.1f, %.1f), wp_idx=%zu",
               vehicle_.c_str(), pause_x_, pause_y_, wp_idx_);
      pause_requested_ = false;
    }
    if (resume_requested_ && paused_)
    {
      return_to_pause_ = true;
      return_x_ = pause_x_;
      return_y_ = pause_y_;
      paused_ = false;
      ROS_INFO("[waypoint_mission] %s RESUMED, returning to odom(%.1f, %.1f)",
               vehicle_.c_str(), return_x_, return_y_);
      resume_requested_ = false;
    }

    switch (state_)
    {
      case TAKEOFF:
      {
        const ros::Time now = ros::Time::now();
        // 错峰起飞：每架按 id 延迟 8 秒，避免 6 机同时爬升/进入航线，
        // 从根源上减少机间避障触发导致的次生碰撞。
        const double stagger = drone_id_ * 8.0;
        if ((now - node_start_).toSec() < stagger)
        {
          publishSmooth(cmd);  // 原地等待
          break;
        }
        if ((now - last_cmd_sent_).toSec() > 1.0)
        {
          sendCmd("OFFBOARD");
          sendCmd("ARM");
          last_cmd_sent_ = now;
        }

        // 起飞阶段水平位置保持：把漂移拉回起飞原点，防止 6 机爬升时互碰。
        if (!takeoff_origin_set_ && pose_z_ > 0.1)
        {
          takeoff_origin_x_ = pose_x_;
          takeoff_origin_y_ = pose_y_;
          takeoff_origin_set_ = true;
        }
        if (takeoff_origin_set_)
        {
          const double err_x = pose_x_ - takeoff_origin_x_;
          const double err_y = pose_y_ - takeoff_origin_y_;
          const double v_east = -takeoff_pos_kp_ * err_x;
          const double v_north = -takeoff_pos_kp_ * err_y;
          const double cy = std::cos(pose_yaw_);
          const double sy = std::sin(pose_yaw_);
          cmd.linear.x = clamp(sy * v_east + cy * v_north, -takeoff_max_horiz_, takeoff_max_horiz_);
          cmd.linear.y = clamp(-cy * v_east + sy * v_north, -takeoff_max_horiz_, takeoff_max_horiz_);
        }

        const double zerr = target_altitude_ - pose_z_;
        cmd.linear.z = clamp(k_z_ * zerr, -max_z_, max_z_);
        publishSmooth(cmd);

        if (std::fabs(zerr) < 0.3)
        {
          state_ = MISSION;
          ROS_INFO("[waypoint_mission] %s altitude reached, starting waypoints",
                   vehicle_.c_str());
        }
        break;
      }

      case MISSION:
      {
        const double zerr = target_altitude_ - pose_z_;
        cmd.linear.z = clamp(k_z_ * zerr, -max_z_, max_z_);

        if (paused_ || !pose_received_ || !scan_received_)
        {
          publishSmooth(cmd);  // 悬停等待（tracking 接管或传感器未就绪）
          break;
        }

        // 当前目标点：恢复时先飞回暂停点，否则按航点顺序。
        double tx = 0.0, ty = 0.0;
        if (return_to_pause_)
        {
          tx = return_x_;
          ty = return_y_;
        }
        else if (wp_idx_ < waypoints_.size())
        {
          tx = waypoints_[wp_idx_].first - spawn_x_;
          ty = waypoints_[wp_idx_].second - spawn_y_;
        }
        else
        {
          // 航点全部完成：悬停，任务结束。
          if (state_ != DONE)
          {
            ROS_INFO("[waypoint_mission] %s all waypoints done, hover",
                     vehicle_.c_str());
            state_ = DONE;
          }
          publishSmooth(cmd);
          break;
        }

        // 到达判定
        const double dist = std::hypot(tx - pose_x_, ty - pose_y_);
        if (return_to_pause_)
        {
          if (dist < wp_reach_tolerance_)
          {
            return_to_pause_ = false;
            wp_idx_ = resume_wp_idx_;
            ROS_INFO("[waypoint_mission] %s back at pause pose, resume wp_idx=%zu",
                     vehicle_.c_str(), wp_idx_);
          }
        }
        else if (dist < wp_reach_tolerance_)
        {
          ++wp_idx_;
          ROS_INFO("[waypoint_mission] %s waypoint %zu reached, next=%zu/%zu",
                   vehicle_.c_str(), wp_idx_ - 1, wp_idx_, waypoints_.size());
        }

        // 目标转到机体系（x 前向，y 左向）。
        const double dx = tx - pose_x_;
        const double dy = ty - pose_y_;
        const double cy = std::cos(pose_yaw_);
        const double sy = std::sin(pose_yaw_);
        const double gx_b = dx * cy + dy * sy;
        const double gy_b = -dx * sy + dy * cy;

        // MPPI 局部规划（避障 + 直飞目标）。
        mppi_.setGoal(gx_b, gy_b);
        mppi_.setScan(scan_.ranges, scan_.angle_min, scan_.angle_increment,
                      scan_.range_max);
        const mppi::Control c = mppi_.plan(smooth_x_, smooth_yaw_);
        cmd.linear.x = c.v;
        cmd.angular.z = c.w;

        // 确定性硬刹车（最后安全网）：MPPI 是采样优化，不能保证 100% 不撞。
        // 用 TTC 式动态刹车距离：速度越快，开始刹车越早。
        // v=4m/s 时 brake_dist=4.8m，< 2.4m 停车；v=2m/s 时 brake_dist=2.4m。
        const double fwd = forwardClearance();
        const double brake_dist = clamp(std::fabs(smooth_x_) * 1.2, 2.0, 8.0);
        if (fwd < 0.5 * brake_dist)
          cmd.linear.x = 0.0;
        else if (fwd < brake_dist)
          cmd.linear.x = std::min(cmd.linear.x, 0.5 + 0.5 * smooth_x_);

        // 机间避障（在 MPPI 输出之上做最后仲裁）。
        avoidPeers(cmd);

        // 转向方向保护：如果 MPPI/机间避障想转过去的方向 clearance < 2m，
        // 放弃转向并减速，防止为避一个障碍而撞上另一个障碍。
        if (std::fabs(cmd.angular.z) > 0.2)
        {
          const double dir = cmd.angular.z > 0.0 ? 30.0 * M_PI / 180.0
                                                 : -30.0 * M_PI / 180.0;
          const double side = sideClearance(dir);
          if (side < 2.0)
          {
            cmd.angular.z = 0.0;
            cmd.linear.x = std::min(cmd.linear.x, 0.5);
          }
        }

        publishSmooth(cmd);
        break;
      }

      case DONE:
      {
        publishSmooth(cmd);
        break;
      }
    }
  }

  // ------------------------------ members ------------------------------
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string vehicle_;
  int drone_id_ = 0;
  double spawn_x_ = -17.0, spawn_y_ = -3.0;

  std::string pose_topic_, scan_topic_, cmd_vel_topic_, cmd_topic_;

  double target_altitude_ = 4.5;
  double max_z_ = 0.8, k_z_ = 0.6;
  double takeoff_pos_kp_ = 0.5, takeoff_max_horiz_ = 1.0;
  double control_rate_ = 20.0;
  double wp_reach_tolerance_ = 2.5;
  double smooth_alpha_ = 0.3;
  double avoid_dist_ = 10.0;

  mppi::Planner mppi_;
  State state_ = TAKEOFF;

  std::vector<std::pair<double, double>> waypoints_;
  size_t wp_idx_ = 0;

  bool pose_received_ = false;
  bool scan_received_ = false;
  double pose_x_ = 0.0, pose_y_ = 0.0, pose_z_ = 0.0, pose_yaw_ = 0.0;
  sensor_msgs::LaserScan scan_;

  // tracking PAUSE/RESUME
  bool paused_ = false;
  bool pause_requested_ = false;
  bool resume_requested_ = false;
  double pause_x_ = 0.0, pause_y_ = 0.0;
  bool return_to_pause_ = false;
  double return_x_ = 0.0, return_y_ = 0.0;
  size_t resume_wp_idx_ = 0;

  // 起飞原点保持
  bool takeoff_origin_set_ = false;
  double takeoff_origin_x_ = 0.0, takeoff_origin_y_ = 0.0;
  ros::Time last_cmd_sent_;
  ros::Time node_start_ = ros::Time::now();

  // 平滑状态
  ros::Time last_smooth_time_;
  double smooth_x_ = 0.0, smooth_yaw_ = 0.0;

  // 机间位置
  struct PeerPose
  {
    std::string ns;
    double x, y;
    ros::Time stamp;
  };
  std::vector<PeerPose> peer_poses_;
  ros::Time last_pose_pub_;

  ros::Subscriber pose_sub_, scan_sub_, mission_control_sub_, drone_pose_sub_;
  ros::Publisher cmd_vel_pub_, cmd_pub_, drone_pose_pub_;
  ros::Timer control_timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_mission_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  WaypointMissionNode node(nh, pnh);
  ros::spin();
  return 0;
}
