#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <string>
#include <utility>
#include <vector>
#include <limits>
#include <XmlRpcValue.h>

#include "auto_explore/astar.hpp"
#include "auto_explore/mppi_planner.hpp"

// Autonomous frontier exploration driven by a 2D lidar + slam_toolbox map.
// Self-contained: it only needs (1) a laser scan topic, (2) the drone pose
// (mavros local_position/pose) and (3) the XTDrone velocity/command interface.
// Low coupling: every topic/frame/param is configurable via ROS params.

namespace
{

double normAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double clamp(double v, double limit)
{
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}

}  // namespace

class ExploreNode
{
public:
  ExploreNode()
    : pnh_("~"),
      state_(TAKEOFF),
      goal_set_(false),
      goal_x_(0), goal_y_(0),
      pose_x_(0), pose_y_(0), pose_z_(0), pose_yaw_(0),
      takeoff_origin_x_(0), takeoff_origin_y_(0),
      takeoff_origin_set_(false),
      map_received_(false), scan_received_(false),
      saw_frontier_(false),
      takeoff_start_(ros::Time::now()),
      last_cmd_sent_(ros::Time(0)),
      last_frontier_time_(ros::Time::now()),
      tf_buffer_(),
      tf_listener_(tf_buffer_)
  {
    // --- topic params ------------------------------------------------------
    std::string vehicle;
    pnh_.param<std::string>("vehicle", vehicle, "typhoon_h480_0");
    vehicle_ = vehicle;
    pnh_.param<std::string>("pose_topic", pose_topic_, "/" + vehicle + "/mavros/local_position/pose");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/" + vehicle + "/scan");
    pnh_.param<std::string>("map_topic", map_topic_, "/map");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/xtdrone/" + vehicle + "/cmd_vel_flu");
    pnh_.param<std::string>("cmd_topic", cmd_topic_, "/xtdrone/" + vehicle + "/cmd");
    pnh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");

    // --- behavior params ---------------------------------------------------
    pnh_.param<double>("target_altitude", target_altitude_, 4.5);
    // 起飞阶段 GPS/位姿水平位置保持：主动把水平漂移拉回起飞原点，
    // 避免 6 机同时爬升时互相漂移碰撞。
    pnh_.param<double>("takeoff_pos_kp", takeoff_pos_kp_, 0.5);
    pnh_.param<double>("takeoff_max_horiz", takeoff_max_horiz_, 1.0);
    pnh_.param<double>("max_lin_x", max_lin_x_, 3.0);
    // 最低巡航速度：正常飞行（非紧急停车）时前向速度的下限，
    // 提高扫图效率，避免一有扰动就掉到很低的蠕动速度。
    pnh_.param<double>("min_lin_x", min_lin_x_, 0.0);
    pnh_.param<double>("max_lin_z", max_lin_z_, 0.8);
    pnh_.param<double>("max_yaw_rate", max_yaw_rate_, 0.4);
    pnh_.param<double>("k_lin", k_lin_, 0.6);
    pnh_.param<double>("accel_limit", accel_limit_, 1.0);
    // 输出平滑：低通 + 加速度限幅，减少探索/追踪时的点头和抖动
    pnh_.param<double>("smooth_alpha", smooth_alpha_, 0.3);
    pnh_.param<double>("smooth_rate_xyz", smooth_rate_xyz_, 1.2);
    pnh_.param<double>("smooth_rate_yaw", smooth_rate_yaw_, 1.0);
    pnh_.param<double>("k_yaw", k_yaw_, 0.8);
    pnh_.param<double>("k_z", k_z_, 0.6);
    pnh_.param<double>("goal_tolerance", goal_tolerance_, 0.5);
    pnh_.param<double>("goal_timeout", goal_timeout_, 20.0);
    pnh_.param<double>("safe_distance", safe_distance_, 6.0);
    pnh_.param<double>("robot_radius", robot_radius_, 0.6);
    pnh_.param<double>("frontier_rate", frontier_rate_, 1.0);
    pnh_.param<double>("control_rate", control_rate_, 20.0);
    pnh_.param<double>("idle_timeout", idle_timeout_, 15.0);
    pnh_.param<double>("takeoff_timeout", takeoff_timeout_, 30.0);
    pnh_.param<int>("num_sectors", num_sectors_, 36);
    pnh_.param<double>("lookahead_dist", lookahead_dist_, 2.0);
    pnh_.param<int>("occupied_thresh", occupied_thresh_, 50);
    pnh_.param<double>("manual_goal_timeout", manual_goal_timeout_, 120.0);
    // Manual waypoint flight: A* global path, string-pulled into a few
    // waypoints, flown fast along straight segments.
    pnh_.param<double>("manual_max_yaw_rate", manual_max_yaw_rate_, 0.8);
    pnh_.param<double>("wp_reach_tolerance", wp_reach_tolerance_, 1.0);
    pnh_.param<double>("manual_brake_dist", manual_brake_dist_, 2.0);
    pnh_.param<double>("manual_replan_interval", manual_replan_interval_, 1.0);
    pnh_.param<double>("plan_inflation", plan_inflation_, 0.3);
    // 手动航点阶段是否把未知栅格当作障碍：true 时 A* 不会规划穿过未扫描
    // 区域的长直线，只走 /map 已知 free + 最新激光已扫清的区域；代价是
    // 初始地图未知时远处航点可能暂时 A* 无路径（控制器会以较低速度朝目标
    // 蠕动并持续扫描，直到地图连通）。
    pnh_.param<bool>("manual_unknown_as_obstacle", manual_unknown_as_obstacle_, true);
    // Transfer altitude: for the first leg (spawn -> first waypoint) and for
    // A* fallback legs, climb a bit (限高 6m，转移高度 ≤6m) so low obstacles /
    // ground rings are less disruptive. Tall buildings still block the 2D A*
    // path, so the backtrack recovery below handles getting trapped.
    pnh_.param<double>("transfer_altitude", transfer_altitude_, 4.5);
    pnh_.param<double>("descend_dist", descend_dist_, 20.0);
    pnh_.param<double>("recovery_timeout", recovery_timeout_, 20.0);
    pnh_.param<double>("backtrack_dist", backtrack_dist_, 15.0);
    pnh_.param<double>("transfer_dist", transfer_dist_, 60.0);
    pnh_.param<double>("plan_clear_radius", plan_clear_radius_, 4.0);
    pnh_.param<double>("avoid_dist", avoid_dist_, 6.0);

    // --- MPPI local planner params ---------------------------------------
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
    pnh_.param<double>("mppi_v_des", mppi_prm.v_des, 3.0);
    pnh_.param<double>("mppi_goal_weight", mppi_prm.goal_weight, 1.2);
    pnh_.param<double>("mppi_obstacle_weight", mppi_prm.obstacle_weight, 40.0);
    pnh_.param<double>("mppi_yaw_weight", mppi_prm.yaw_weight, 0.05);
    pnh_.param<double>("mppi_speed_weight", mppi_prm.speed_weight, 0.4);
    pnh_.param<double>("mppi_field_radius", mppi_prm.field_radius, 18.0);
    pnh_.param<double>("mppi_field_res", mppi_prm.field_res, 0.15);
    pnh_.param<double>("mppi_safe_radius", mppi_prm.safe_radius, 2.5);
    mppi_.configure(mppi_prm);

    pnh_.param<std::string>("map_save_path", map_save_path_,
                            "/home/lishu2/2025_ZZU_FLY/my_maps/" + vehicle);

    // --- hardcoded waypoints (world/ENU frame) ----------------------------
    // `waypoints` is a list of [x, y] pairs expressed in the mavros EKF world
    // frame (same frame as `odom` in our tf tree). They are converted into the
    // SLAM map frame once tf is available and then flown point-to-point in
    // order. Empty list => autonomous frontier exploration as usual.
    XmlRpc::XmlRpcValue wps;
    if (pnh_.getParam("waypoints", wps) &&
        wps.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < wps.size(); ++i)
      {
        if (wps[i].getType() == XmlRpc::XmlRpcValue::TypeArray && wps[i].size() >= 2)
          preset_waypoints_.push_back(
              std::make_pair(static_cast<double>(wps[i][0]),
                             static_cast<double>(wps[i][1])));
      }
      ROS_INFO("[auto_explore] loaded %zu preset waypoints (world frame)",
               preset_waypoints_.size());
    }

    // --- subs / pubs / timers ---------------------------------------------
    pose_sub_  = nh_.subscribe(pose_topic_, 1, &ExploreNode::poseCb, this);
    scan_sub_  = nh_.subscribe(scan_topic_, 1, &ExploreNode::scanCb, this);
    map_sub_   = nh_.subscribe(map_topic_, 1, &ExploreNode::mapCb, this);
    manual_goal_sub_ = nh_.subscribe(goal_topic_, 1, &ExploreNode::manualGoalCb, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    cmd_pub_     = nh_.advertise<std_msgs::String>(cmd_topic_, 1);
    marker_pub_  = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
    // Peer exploration-goal sharing for multi-drone dedup: every drone
    // publishes its current frontier goal as a Marker on /explore_goals and
    // subscribes to all peers, so cluster selection skips regions already
    // being explored by another drone.
    explore_goal_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/explore_goals", 1);
    peer_goal_sub_ = nh_.subscribe("/explore_goals", 10,
                                   &ExploreNode::peerGoalCb, this);
    peer_goals_.clear();

    // Peer pose sharing for collision avoidance: publish own map-frame pose
    // once per control cycle, subscribe to every peer's pose.
    drone_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/drone_poses", 1);
    drone_pose_sub_ = nh_.subscribe("/drone_poses", 10,
                                    &ExploreNode::dronePoseCb, this);
    peer_poses_.clear();

    // tracking 节点在接管控制权前后会发布 PAUSE / RESUME：
    //   PAUSE  -> 记录当前航点任务位置，暂停航点推进（tracking 接管）
    //   RESUME -> 把记录的位置插回队首，先飞回暂停点再继续原航点任务
    mission_control_sub_ = nh_.subscribe("/" + vehicle_ + "/mission/control", 1,
                                         &ExploreNode::missionControlCb, this);

    control_timer_  = nh_.createTimer(ros::Duration(1.0 / control_rate_), &ExploreNode::controlCb, this);
    frontier_timer_ = nh_.createTimer(ros::Duration(1.0 / frontier_rate_), &ExploreNode::frontierCb, this);

    ROS_INFO("[auto_explore] vehicle=%s | scan=%s | map=%s | cmd_vel=%s | save=%s",
             vehicle.c_str(), scan_topic_.c_str(), map_topic_.c_str(),
             cmd_vel_topic_.c_str(), map_save_path_.c_str());
  }

private:
  enum State { TAKEOFF, EXPLORE, DONE };

  // ------------------------------------------------------------------ callbacks
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    pose_z_ = msg->pose.position.z;
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                      msg->pose.orientation.z, msg->pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, pose_yaw_);
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    scan_ = *msg;
    scan_received_ = true;
  }

  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    map_ = *msg;
    map_received_ = true;
  }

  // Append a manually-specified goal (rviz "2D Nav Goal") to the FIFO queue.
  // Points are expected in the SLAM map frame; we transform into map_frame_
  // just in case rviz is configured with a different fixed frame.
  void manualGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    double gx = msg->pose.position.x;
    double gy = msg->pose.position.y;
    try
    {
      geometry_msgs::PoseStamped in = *msg;
      geometry_msgs::PoseStamped out;
      tf_buffer_.transform(in, out, map_frame_, ros::Duration(0.5));
      gx = out.pose.position.x;
      gy = out.pose.position.y;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(5.0, "[auto_explore] goal tf to %s failed: %s",
                        map_frame_.c_str(), ex.what());
    }
    manual_goals_.push_back(std::make_pair(gx, gy));
    ROS_INFO("[auto_explore] queued manual goal (%.1f, %.1f), %zu pending",
             gx, gy, manual_goals_.size());
  }

  // Collect peers' current frontier goals (Marker on /explore_goals, one per
  // drone, ns=vehicle name). Used to avoid selecting an already-claimed
  // frontier cluster. Only goals newer than 3 s count (stale goals released).
  void peerGoalCb(const visualization_msgs::Marker::ConstPtr& msg)
  {
    if (msg->ns == vehicle_) return;  // ignore own goal
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    if (age > 3.0) return;
    bool updated = false;
    for (PeerGoal& pg : peer_goals_)
    {
      if (pg.ns == msg->ns)
      {
        pg.x = msg->pose.position.x;
        pg.y = msg->pose.position.y;
        pg.stamp = msg->header.stamp;
        updated = true;
        break;
      }
    }
    if (!updated)
      peer_goals_.push_back({msg->ns, msg->pose.position.x,
                             msg->pose.position.y, msg->header.stamp});
    // Drop stale peers so claimed regions are released when a peer finishes.
    const ros::Time now = ros::Time::now();
    for (auto it = peer_goals_.begin(); it != peer_goals_.end();)
    {
      if ((now - it->stamp).toSec() > 3.0) it = peer_goals_.erase(it);
      else ++it;
    }
  }

  // Track every peer drone's map-frame position (for collision avoidance).
  // frame_id carries the vehicle name (typhoon_h480_<id>); position is in the
  // shared map frame (published by controlCb). Ignore our own broadcast.
  void dronePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (msg->header.frame_id.empty()) return;
    if (msg->header.frame_id == vehicle_) return;  // own pose
    const ros::Duration age = ros::Time::now() - msg->header.stamp;
    if (age.toSec() > 1.0) return;  // stale
    bool updated = false;
    for (PeerPose& pp : peer_poses_)
    {
      if (pp.ns == msg->header.frame_id)  // frame_id carries the vehicle name
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
    const ros::Time now = ros::Time::now();
    for (auto it = peer_poses_.begin(); it != peer_poses_.end();)
    {
      if ((now - it->stamp).toSec() > 1.0) it = peer_poses_.erase(it);
      else ++it;
    }
  }

  // tracking 节点的 PAUSE/RESUME 请求。只在这里设置标志位，实际的位姿
  // 记录与航点回插在 controlCb / frontierCb 中处理，避免在回调里做 tf。
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

  // Convert the hardcoded preset waypoints from the mavros EKF world frame
  // (odom) into the SLAM map frame and enqueue them as manual goals. Called
  // once, after the map/tf is up, so the odom->map transform is settled.
  // Returns false (to retry next cycle) if any transform is unavailable.
  bool loadPresetWaypoints()
  {
    for (const auto& wp : preset_waypoints_)
    {
      geometry_msgs::PoseStamped in;
      in.header.frame_id = odom_frame_;
      in.header.stamp = ros::Time(0);
      in.pose.position.x = wp.first;
      in.pose.position.y = wp.second;
      in.pose.position.z = 0.0;
      in.pose.orientation.w = 1.0;
      try
      {
        geometry_msgs::PoseStamped out;
        tf_buffer_.transform(in, out, map_frame_, ros::Duration(1.0));
        manual_goals_.push_back(
            std::make_pair(out.pose.position.x, out.pose.position.y));
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(5.0, "[auto_explore] preset waypoint tf %s->%s failed: %s",
                          odom_frame_.c_str(), map_frame_.c_str(), ex.what());
        return false;
      }
    }
    ROS_INFO("[auto_explore] enqueued %zu preset waypoints (map frame)",
             manual_goals_.size());
    return true;
  }

  // ------------------------------------------------------------------ helpers
  void sendCmd(const std::string& cmd)
  {
    std_msgs::String m;
    m.data = cmd;
    cmd_pub_.publish(m);
  }

  void setGoal(double gx, double gy, const ros::Time& now)
  {
    goal_x_ = gx;
    goal_y_ = gy;
    goal_set_ = true;
    goal_stamp_ = now;

    visualization_msgs::Marker mk;
    mk.header.frame_id = map_frame_;
    mk.header.stamp = now;
    mk.ns = "auto_explore";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = gx;
    mk.pose.position.y = gy;
    mk.pose.position.z = 0.5;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = 1.0;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.2;
    mk.color.b = 0.2;
    marker_pub_.publish(mk);

    // Share this goal with peers for exploration dedup (explore phase only;
    // manual waypoint goals keep their own marker and are not advertised).
    if (manual_goals_.empty())
    {
      visualization_msgs::Marker gm = mk;
      gm.ns = vehicle_;
      gm.id = 1;
      gm.color.r = 0.2;
      gm.color.g = 0.8;
      gm.color.b = 0.2;
      explore_goal_pub_.publish(gm);
    }
  }

  // Project the latest filtered 2D laser scan into the A* window grid before
  // inflation. The shared /map is fused by scan_map_fuser at a lower rate and
  // may not yet contain an obstacle the lidar has just seen; this gives A*
  // an up-to-date local obstacle layer without waiting for the next /map.
  void fuseLatestScanToGrid(std::vector<int8_t>& grid, int gw, int gh,
                            int minx, int miny,
                            double mx, double my, double myaw)
  {
    if (!scan_received_ || scan_.ranges.empty()) return;

    const double res = map_.info.resolution;
    const double ox = map_.info.origin.position.x;
    const double oy = map_.info.origin.position.y;

    const int sx = static_cast<int>(std::floor((mx - ox) / res));
    const int sy = static_cast<int>(std::floor((my - oy) / res));
    const int lx = sx - minx, ly = sy - miny;  // laser origin in window coords

    const float rmax = scan_.range_max;
    for (size_t k = 0; k < scan_.ranges.size(); ++k)
    {
      const float r = scan_.ranges[k];
      if (!(r >= scan_.range_min && r <= rmax) || !std::isfinite(r))
        continue;

      const double a = scan_.angle_min + k * scan_.angle_increment + myaw;
      const double ex = mx + r * std::cos(a);
      const double ey = my + r * std::sin(a);
      const int ex_cell = static_cast<int>(std::floor((ex - ox) / res));
      const int ey_cell = static_cast<int>(std::floor((ey - oy) / res));
      const int lex = ex_cell - minx, ley = ey_cell - miny;

      // The ray proves these cells are free at the current flight height;
      // the hit cell is marked occupied just below (unless the ray reached
      // max range, which means "no echo" and therefore only a clear ray).
      clearWindowLine(grid, gw, gh, lx, ly, lex, ley, 0);

      if (r < rmax - 0.05f &&
          lex >= 0 && lex < gw && ley >= 0 && ley < gh)
        grid[ley * gw + lex] = 1;
    }
  }

  // Bresenham line over the local A* window grid, clipped to window bounds.
  void clearWindowLine(std::vector<int8_t>& grid, int gw, int gh,
                       int x0, int y0, int x1, int y1, int8_t val)
  {
    const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (true)
    {
      if (x >= 0 && x < gw && y >= 0 && y < gh)
        grid[y * gw + x] = val;
      if (x == x1 && y == y1) break;
      const int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  // Recompute the global A* path from the drone to the current goal, both in
  // the SLAM map frame. Occupied cells block the path. Unknown cells are
  // traversable by default (frontier exploration needs them), but manual
  // waypoint flight can set unknown_as_obstacle=true so the planned path only
  // uses known-free space and never shoots through an unmapped building row.
  // `inflation` (meters) optionally dilates obstacles so the path keeps a
  // safety margin from walls; manual waypoint flight uses it (0.6 m default)
  // so the drone can fly fast without clipping corners, while exploration
  // keeps 0.0 to preserve the long-tested behaviour.
  void planPath(double mx, double my, double myaw, double gx, double gy,
                double inflation = 0.0, bool unknown_as_obstacle = false)
  {
    path_.clear();
    if (!map_received_) return;

    const nav_msgs::OccupancyGrid& m = map_;
    const int w = m.info.width;
    const int h = m.info.height;
    const double res = m.info.resolution;
    const double ox = m.info.origin.position.x;
    const double oy = m.info.origin.position.y;

    const int sx = static_cast<int>(std::floor((mx - ox) / res));
    const int sy = static_cast<int>(std::floor((my - oy) / res));
    const int gxc = static_cast<int>(std::floor((gx - ox) / res));
    const int gyc = static_cast<int>(std::floor((gy - oy) / res));

    // Plan on a padded window that always contains start AND goal. Cells
    // outside the current (still-growing) /map are treated as free/unknown, so
    // A* can route toward far waypoints the moment they are queued; the 1 Hz
    // replan refines the plan as the map grows, and the lidar-reactive
    // controller is the short-range safety net. Previously an off-map goal
    // made A* skip entirely and the drone fell back to blind straight flight
    // that wandered among buildings (waypoint timeouts).
    const int PAD = 60;  // cells (6 m) of margin around start/goal
    const int minx = std::min(sx, gxc) - PAD, maxx = std::max(sx, gxc) + PAD;
    const int miny = std::min(sy, gyc) - PAD, maxy = std::max(sy, gyc) + PAD;
    const int gw = maxx - minx + 1, gh = maxy - miny + 1;
    const int MAXS = 2600;  // safety cap; must cover full map (2300 cells) so
                            // long road legs are not skipped
    if (gw > MAXS || gh > MAXS)
    {
      ROS_WARN_THROTTLE(3.0,
        "[auto_explore] A* window %dx%d too large (goal %d,%d), skip",
        gw, gh, gxc, gyc);
      return;
    }

    std::vector<int8_t> grid(gw * gh, 0);  // default free (off-map / unknown)
    for (int y = std::max(0, miny); y <= std::min(h - 1, maxy); ++y)
    {
      for (int x = std::max(0, minx); x <= std::min(w - 1, maxx); ++x)
      {
        const int8_t v = m.data[y * w + x];
        if (v > occupied_thresh_)
          grid[(y - miny) * gw + (x - minx)] = 1;
        else if (unknown_as_obstacle && v < 0)
          grid[(y - miny) * gw + (x - minx)] = 1;  // manual: unmapped blocks
      }
    }

    // Fuse the latest own filtered laser scan into the local grid before
    // inflation, so A* sees obstacles that are not yet in /map (the shared
    // map is fused asynchronously and may lag a few hundred ms). Laser rays
    // also clear unknown cells they just proved free, so unknown-as-obstacle
    // mode does not block the drone inside its current scanned bubble.
    fuseLatestScanToGrid(grid, gw, gh, minx, miny, mx, my, myaw);

    // Optional circular obstacle inflation (manual planning only).
    if (inflation > 0.0)
    {
      const int ir = std::max(1, static_cast<int>(std::ceil(inflation / res)));
      const int ir2 = ir * ir;
      std::vector<int8_t> inflated(grid);
      for (int y = 0; y < gh; ++y)
        for (int x = 0; x < gw; ++x)
        {
          if (grid[y * gw + x] != 1) continue;
          for (int dy = -ir; dy <= ir; ++dy)
            for (int dx = -ir; dx <= ir; ++dx)
            {
              if (dx * dx + dy * dy > ir2) continue;  // circular footprint
              const int ax = x + dx, ay = y + dy;
              if (ax >= 0 && ax < gw && ay >= 0 && ay < gh)
                inflated[ay * gw + ax] = 1;
            }
        }
      grid.swap(inflated);
    }

    // Carve the robot footprint free around the start (its own lidar marks the
    // cell it stands on as occupied) and a small pocket around the goal, so a
    // real path is not blocked by self-hits.
    const int rc = std::max(1, static_cast<int>(std::ceil(plan_clear_radius_ / res)));
    const int lx = sx - minx, ly = sy - miny;    // start in window coords
    const int lgx = gxc - minx, lgy = gyc - miny;  // goal in window coords
    for (int dy = -rc; dy <= rc; ++dy)
      for (int dx = -rc; dx <= rc; ++dx)
      {
        const int ax = lx + dx, ay = ly + dy;
        if (ax >= 0 && ax < gw && ay >= 0 && ay < gh)
          grid[ay * gw + ax] = 0;
      }
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx)
      {
        const int bx = lgx + dx, by = lgy + dy;
        if (bx >= 0 && bx < gw && by >= 0 && by < gh)
          grid[by * gw + bx] = 0;
      }

    path_ = astar::plan(grid, gw, gh, {lx, ly}, {lgx, lgy});
    // astar returns cells in window coords; convert back to real map cells so
    // publishPathMarker / simplifyPathToWaypoints keep working unchanged.
    for (astar::Cell& c : path_) { c.x += minx; c.y += miny; }
    if (path_.empty())
      ROS_WARN_THROTTLE(3.0,
        "[auto_explore] A* no path: start(%d,%d) goal(%d,%d) window %dx%d",
        sx, sy, gxc, gyc, gw, gh);

    publishPathMarker();
  }

  // Draw the current global path as a green LINE_STRIP for rviz debugging.
  void publishPathMarker()
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = map_frame_;
    mk.header.stamp = ros::Time::now();
    mk.ns = "auto_explore_path";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.scale.x = 0.1;
    mk.color.a = 1.0;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;

    if (path_.empty())
    {
      mk.action = visualization_msgs::Marker::DELETE;
      marker_pub_.publish(mk);
      return;
    }

    mk.action = visualization_msgs::Marker::ADD;
    const double ox = map_.info.origin.position.x;
    const double oy = map_.info.origin.position.y;
    const double res = map_.info.resolution;
    mk.points.reserve(path_.size());
    for (const astar::Cell& c : path_)
    {
      geometry_msgs::Point p;
      p.x = ox + (c.x + 0.5) * res;
      p.y = oy + (c.y + 0.5) * res;
      p.z = 0.5;
      mk.points.push_back(p);
    }
    marker_pub_.publish(mk);
  }

  // True when the straight segment between two world points crosses no
  // occupied cell (Bresenham over the occupancy grid). Free and unknown cells
  // are traversable; anything above occupied_thresh_ blocks. A tiny number of
  // isolated occupied cells (other drones' transient lidar ghosts, one-off
  // stray hits) do NOT block — only a genuine wall registers as hundreds of
  // consecutive cells. Cells outside the current /map are unknown, hence
  // clear — the drone flies toward far goals through unmapped space while the
  // 1 Hz replan + lidar-reactive controller handle surprises.
  bool segmentClear(double ax, double ay, double bx, double by)
  {
    if (!map_received_) return false;
    const nav_msgs::OccupancyGrid& m = map_;
    const int w = m.info.width, h = m.info.height;
    const double res = m.info.resolution;
    const double ox = m.info.origin.position.x, oy = m.info.origin.position.y;
    int x0, y0, x1, y1;
    x0 = static_cast<int>(std::floor((ax - ox) / res));
    y0 = static_cast<int>(std::floor((ay - oy) / res));
    x1 = static_cast<int>(std::floor((bx - ox) / res));
    y1 = static_cast<int>(std::floor((by - oy) / res));

    const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;
    int blocked = 0, total = 0;
    while (true)
    {
      if (x >= 0 && x < w && y >= 0 && y < h)
      {
        ++total;
        if (m.data[y * w + x] > occupied_thresh_)
        {
          ++blocked;
          // Any run of >= 3 consecutive blocked cells = real obstacle.
          // Isolated ghosts (1-2 cells) don't stop a transfer leg.
          if (blocked >= 3) return false;
        }
        else
        {
          blocked = 0;  // reset run on any free cell
        }
      }
      if (x == x1 && y == y1) break;
      const int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
    return true;
  }

  // Turn the A* cell path into a compact list of world-frame waypoints whose
  // straight segments are all obstacle-free ("string pulling"): from the start
  // keep reaching for the furthest path point still visible in a straight
  // line, so the drone flies long fast segments instead of creeping cell by
  // cell. Result lives in the SLAM map frame.
  void simplifyPathToWaypoints()
  {
    manual_wps_.clear();
    if (path_.size() < 2) return;

    const nav_msgs::OccupancyGrid& m = map_;
    const double res = m.info.resolution;
    const double ox = m.info.origin.position.x, oy = m.info.origin.position.y;
    std::vector<std::pair<double, double>> pts;
    pts.reserve(path_.size());
    for (const astar::Cell& c : path_)
      pts.emplace_back(ox + (c.x + 0.5) * res, oy + (c.y + 0.5) * res);

    manual_wps_.push_back(pts.front());
    size_t i = 0;  // index (in pts) of the last stored waypoint
    while (i + 1 < pts.size())
    {
      size_t j = i + 1;
      for (size_t k = i + 2; k < pts.size(); ++k)
      {
        if (segmentClear(pts[i].first, pts[i].second, pts[k].first, pts[k].second))
          j = k;  // furthest visible point along this leg
        else
          break;  // anything blocking between => stop extending this leg
      }
      manual_wps_.push_back(pts[j]);
      i = j;
    }
  }

  // After a replan the path starts from the drone's current cell, so re-derive
  // the waypoint index: continue from the first waypoint not already under the
  // drone. Prevents the index regressing when the plan refreshes in place.
  void rebaseWaypointIndex(double mx, double my)
  {
    size_t i = 0;
    while (i < manual_wps_.size() &&
           std::hypot(manual_wps_[i].first - mx, manual_wps_[i].second - my) <
               wp_reach_tolerance_)
      ++i;
    wp_idx_ = i;
  }

  // Peer collision avoidance: if another drone is closer than avoid_dist_,
  // slow down and steer away from it (override yaw toward the escape
  // direction). Applied right before the twist is published, in both manual
  // waypoint flight and frontier exploration. Peers broadcast their map-frame
  // pose on /drone_poses at the control rate; a stale (>1 s) peer is dropped.
  // Avoidance is suppressed while parked at the spawn cluster (dist to goal
  // < 6 m): there the drones sit 3 m apart and would otherwise block each
  // other from taking off.
  void avoidPeers(geometry_msgs::Twist& cmd, double mx, double my, double myaw)
  {
    // Never avoid inside the spawn cluster: the six drones start 3 m apart
    // (well inside avoid_dist_) and mutual avoidance there would lock them in
    // place, unable to take off. Only engage once away from spawn.
    if (!have_spawn_ || std::hypot(mx - spawn_x_, my - spawn_y_) < 12.0)
      return;
    double best_d2 = avoid_dist_ * avoid_dist_;
    double bx = 0.0, by = 0.0;
    const ros::Time now = ros::Time::now();
    for (const PeerPose& pp : peer_poses_)
    {
      if ((now - pp.stamp).toSec() > 0.5) continue;  // fresher than 0.5 s
      const double ddx = pp.x - mx, ddy = pp.y - my;
      const double d2 = ddx * ddx + ddy * ddy;
      if (d2 < best_d2) { best_d2 = d2; bx = ddx; by = ddy; }
    }
    if (best_d2 >= avoid_dist_ * avoid_dist_) return;

    const double d = std::sqrt(best_d2);
    // Escape heading = away from the peer.
    const double escape = std::atan2(-by, -bx);
    const double yaw_err = normAngle(escape - myaw);
    // Strongly prioritize avoidance: hard-turn toward escape and cut speed
    // progressively from avoid_dist_ down (full stop inside 1.5 m).
    cmd.angular.z = clamp(k_yaw_ * yaw_err * 3.0, manual_max_yaw_rate_);
    const double closeness = 1.0 - d / avoid_dist_;  // 0 far -> 1 touching
    cmd.linear.x *= (1.0 - 0.9 * closeness);
    if (d < 1.5) cmd.linear.x = 0.0;
  }

  // MPPI local planner: build a body-frame obstacle distance field from the
  // latest filtered scan, transform the global goal into the body frame and
  // plan one control cycle. Returns body-frame forward speed and yaw rate.
  // Replaces the old sector-based reactive heading + hand-tuned speed ramp.
  void applyMppiControl(double mx, double my, double myaw, double tx, double ty,
                        geometry_msgs::Twist& cmd)
  {
    if (!scan_received_)
    {
      // No scan yet: yaw toward the goal, no forward motion.
      const double gy = std::atan2(ty - my, tx - mx);
      cmd.angular.z = clamp(k_yaw_ * normAngle(gy - myaw), manual_max_yaw_rate_);
      return;
    }

    // Goal in body frame: x forward, y left.
    const double dx = tx - mx;
    const double dy = ty - my;
    const double cos_yaw = std::cos(myaw);
    const double sin_yaw = std::sin(myaw);
    const double gx_b = dx * cos_yaw + dy * sin_yaw;
    const double gy_b = -dx * sin_yaw + dy * cos_yaw;

    mppi_.setGoal(gx_b, gy_b);
    mppi_.setScan(scan_.ranges, scan_.angle_min, scan_.angle_increment,
                  scan_.range_max);

    const double v_now = last_lin_x_;      // current smoothed forward speed
    const double w_now = last_smooth_yaw_; // current smoothed yaw rate
    const mppi::Control c = mppi_.plan(v_now, w_now);

    cmd.linear.x = c.v;
    cmd.angular.z = c.w;
  }

  // Manual fast waypoint flight: aim at the current pruned A* waypoint and fly
  // straight at (near) full speed; brake only for unexpected obstacles (the
  // planned segment is already collision-free). Falls back to the reactive
  // straight-line behaviour when no A* path exists yet (map not ready, goal
  // off-map, or unreachable).
  void manualFlightControl(geometry_msgs::Twist& cmd, double mx, double my,
                           double myaw)
  {
    // Consume waypoints already under the drone.
    while (wp_idx_ < manual_wps_.size() &&
           std::hypot(manual_wps_[wp_idx_].first - mx,
                      manual_wps_[wp_idx_].second - my) < wp_reach_tolerance_)
      ++wp_idx_;
    if (wp_idx_ >= manual_wps_.size())
    {
      manual_wps_.clear();  // all waypoints consumed; frontierCb pops the goal
      wp_idx_ = 0;
    }

    double tx = goal_x_, ty = goal_y_;
    if (!manual_wps_.empty())
    {
      tx = manual_wps_[wp_idx_].first;
      ty = manual_wps_[wp_idx_].second;
    }
    else if (!transfer_mode_ && have_retreat_ &&
             std::hypot(goal_x_ - mx, goal_y_ - my) > backtrack_dist_ &&
             (ros::Time::now() - last_astar_ok_).toSec() > recovery_timeout_)
    {
      // A* has failed for a while AND we are far from the goal (trapped in a
      // building cluster): backtrack to the last pose where a path existed,
      // instead of blind-creeping toward the goal through the cluster. Near
      // the goal we just creep the last stretch at walking pace. The 1 Hz
      // planner keeps retrying; once it recovers, normal flight resumes.
      // (transfer_mode_ means the straight line was verified clear — never
      //  backtrack out of a fast transfer leg.)
      tx = retreat_x_;
      ty = retreat_y_;
      ROS_WARN_THROTTLE(3.0, "[auto_explore] A* stuck, backtracking to (%.1f, %.1f)",
                        retreat_x_, retreat_y_);
    }

    std::vector<double> sector_min;
    buildSectors(sector_min);
    const double fwd = forwardClearance(sector_min);
    const bool in_spawn =
        !have_spawn_ || std::hypot(mx - spawn_x_, my - spawn_y_) < 12.0;

    // MPPI local planner replaces the old sector-based heading/speed ladder.
    // `fwd` and `sector_min` are still used below only to decide when to ask
    // the global A* planner for an early reroute.
    applyMppiControl(mx, my, myaw, tx, ty, cmd);

    // The lidar already sees something close: ask the 1 Hz planner for a
    // reroute immediately (the new obstacle is not in the stale /map plan).
    // In transfer (blind-leg) mode request it much sooner — 20 m out — so A*
    // reroutes before the drone reaches the wall.
    const double replan_dist =
        (transfer_mode_ && manual_wps_.empty())
            ? std::max(manual_brake_dist_, 20.0)
            : std::max(manual_brake_dist_, 15.0);
    if (!in_spawn && fwd < replan_dist)
      replan_requested_ = true;

    cmd.linear.x = rampLinX(cmd.linear.x);
    avoidPeers(cmd, mx, my, myaw);  // multi-drone collision avoidance
  }

  // Apply low-pass + rate limiting to the whole cmd_vel before publishing.
  // This reduces frequent pitching/nodding in both exploration and tracking.
  void publishSmooth(geometry_msgs::Twist& cmd)
  {
    const ros::Time now = ros::Time::now();
    double dt = (last_smooth_time_.is_zero()) ? (1.0 / control_rate_)
                                              : (now - last_smooth_time_).toSec();
    last_smooth_time_ = now;
    dt = std::max(0.01, std::min(dt, 0.5));

    // Low-pass
    smooth_x_ += smooth_alpha_ * (cmd.linear.x - smooth_x_);
    smooth_y_ += smooth_alpha_ * (cmd.linear.y - smooth_y_);
    smooth_z_ += smooth_alpha_ * (cmd.linear.z - smooth_z_);
    smooth_yaw_ += smooth_alpha_ * (cmd.angular.z - smooth_yaw_);

    // Rate limit
    const double max_xyz = smooth_rate_xyz_ * dt;
    const double max_yaw = smooth_rate_yaw_ * dt;
    smooth_x_ = clamp(smooth_x_ - last_smooth_x_, max_xyz) + last_smooth_x_;
    smooth_y_ = clamp(smooth_y_ - last_smooth_y_, max_xyz) + last_smooth_y_;
    smooth_z_ = clamp(smooth_z_ - last_smooth_z_, max_xyz) + last_smooth_z_;
    smooth_yaw_ = clamp(smooth_yaw_ - last_smooth_yaw_, max_yaw) + last_smooth_yaw_;

    cmd.linear.x = smooth_x_;
    cmd.linear.y = smooth_y_;
    cmd.linear.z = smooth_z_;
    cmd.angular.z = smooth_yaw_;

    last_smooth_x_ = smooth_x_;
    last_smooth_y_ = smooth_y_;
    last_smooth_z_ = smooth_z_;
    last_smooth_yaw_ = smooth_yaw_;

    cmd_vel_pub_.publish(cmd);
  }

  // Smooth the forward speed setpoint: cap ACCELERATION to accel_limit_ so the
  // drone builds up speed gently (turning/braking slowdowns stay instant for
  // safety — only speed *increases* are rate-limited).
  double rampLinX(double target)
  {
    const double dt = 1.0 / control_rate_;
    const double dv_max = accel_limit_ * dt;
    if (target > last_lin_x_ + dv_max)
      target = last_lin_x_ + dv_max;
    last_lin_x_ = target;
    return target;
  }

  // Robot pose in the SLAM map frame (via tf). The mavros pose lives in the
  // EKF "map" frame; slam_toolbox rotates/offsets its own "map" frame relative
  // to odom, so frontier goals (in the SLAM map frame) must be compared against
  // a pose in the SAME frame — otherwise heading is wrong and the drone spins.
  bool getMapPose(double& mx, double& my, double& myaw)
  {
    try
    {
      geometry_msgs::TransformStamped ts =
          tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));
      mx = ts.transform.translation.x;
      my = ts.transform.translation.y;
      tf2::Quaternion q(ts.transform.rotation.x, ts.transform.rotation.y,
                        ts.transform.rotation.z, ts.transform.rotation.w);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, myaw);
      if (!have_spawn_)
      {
        have_spawn_ = true;
        spawn_x_ = mx;
        spawn_y_ = my;
      }
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(5.0, "[auto_explore] tf %s->%s failed: %s",
                        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  // Compute min range per angular sector from the latest scan.
  void buildSectors(std::vector<double>& sector_min)
  {
    sector_min.assign(num_sectors_, std::numeric_limits<double>::max());
    if (!scan_received_) return;
    const double sector_ang = 2.0 * M_PI / num_sectors_;
    for (size_t i = 0; i < scan_.ranges.size(); ++i)
    {
      const double r = scan_.ranges[i];
      if (r < scan_.range_min || r > scan_.range_max) continue;
      const double a = scan_.angle_min + i * scan_.angle_increment;
      int idx = static_cast<int>(std::floor((a + M_PI) / sector_ang));
      if (idx < 0) idx = 0;
      if (idx >= num_sectors_) idx = num_sectors_ - 1;
      if (r < sector_min[idx]) sector_min[idx] = r;
    }
  }

  int angleToSector(double a) const
  {
    const double sector_ang = 2.0 * M_PI / num_sectors_;
    int idx = static_cast<int>(std::floor((a + M_PI) / sector_ang));
    if (idx < 0) idx = 0;
    if (idx >= num_sectors_) idx = num_sectors_ - 1;
    return idx;
  }

  double sectorToAngle(int sector) const
  {
    const double sector_ang = 2.0 * M_PI / num_sectors_;
    return -M_PI + (sector + 0.5) * sector_ang;
  }

  // Reactive heading: go to goal if that direction is clear, else follow the
  // largest free gap (this is what produces road/wall-following behavior).
  // `cone` >= 1 additionally requires the goal direction's neighbours to be
  // clear too, so a wall sitting just off the direct bearing also triggers a
  // detour instead of being clipped at high speed. Exploration keeps cone=0
  // (long-tested behaviour); manual waypoint flight uses cone=1.
  double chooseHeading(double goal_dir, const std::vector<double>& sector_min,
                       int cone = 0)
  {
    const int goal_idx = angleToSector(goal_dir);

    // 1) 目标方向及其相邻扇区都足够空旷：保持飞向目标。
    bool clear = true;
    for (int d = -cone; d <= cone; ++d)
    {
      const int s = (goal_idx + d + num_sectors_) % num_sectors_;
      if (sector_min[s] <= safe_distance_) { clear = false; break; }
    }
    if (clear)
    {
      // 目标方向虽然安全，但如果远处(avoid_dist_以内)已有障碍，提前轻微
      // 偏向往更空的一侧，避免贴脸才转弯。
      const double fwd = sector_min[goal_idx];
      if (fwd < avoid_dist_)
      {
        const int l = (goal_idx - 2 + num_sectors_) % num_sectors_;
        const int r = (goal_idx + 2) % num_sectors_;
        const double push = (avoid_dist_ - fwd) / avoid_dist_;  // 0..1
        if (sector_min[l] >= sector_min[r])
          return normAngle(goal_dir + push * 0.6);
        else
          return normAngle(goal_dir - push * 0.6);
      }
      return goal_dir;
    }

    // 2) 目标方向被挡：在激光扫描的所有“连续空闲扇区段”里，选一个
    //    既靠近目标方向、又有足够 clearance 的缺口，而不是简单选最大缺口。
    //    优先选择宽度 >= 2 个扇区的缺口，避免钻进太窄的缝。
    struct Gap
    {
      int start;
      int len;
      double min_range;
      double center;
    };
    auto collect_gaps = [&](int min_len) {
      std::vector<Gap> gaps;
      int cur_start = 0, cur_len = 0;
      double cur_min = std::numeric_limits<double>::max();
      for (int i = 0; i <= 2 * num_sectors_; ++i)
      {
        const int idx = i % num_sectors_;
        const bool free = (i < 2 * num_sectors_) &&
                          (sector_min[idx] > robot_radius_ + 1.0);
        if (free)
        {
          if (cur_len == 0) { cur_start = i; cur_min = sector_min[idx]; }
          ++cur_len;
          cur_min = std::min(cur_min, sector_min[idx]);
        }
        else if (cur_len > 0)
        {
          if (cur_len >= min_len)
          {
            const int start_mod = cur_start % num_sectors_;
            const int center = (start_mod + cur_len / 2) % num_sectors_;
            gaps.push_back({start_mod, cur_len, cur_min, sectorToAngle(center)});
          }
          cur_len = 0;
          cur_min = std::numeric_limits<double>::max();
        }
      }
      return gaps;
    };

    std::vector<Gap> gaps = collect_gaps(2);   // 至少 2 个扇区宽
    if (gaps.empty())
      gaps = collect_gaps(1);                  // 实在没有，再接受单扇区缺口

    if (gaps.empty())
      return goal_dir;  // 没有可通行缺口：保持目标方向，由速度刹车兜底

    // 选择最优缺口：越靠近目标方向越好，clearance 越大越好。
    double best_score = std::numeric_limits<double>::max();
    double best_center = gaps.front().center;
    for (const Gap& g : gaps)
    {
      const double dist_to_goal = std::fabs(normAngle(g.center - goal_dir));
      const double score = dist_to_goal * 2.5 - 0.12 * std::min(g.min_range, 20.0);
      if (score < best_score)
      {
        best_score = score;
        best_center = g.center;
      }
    }
    return best_center;
  }

  // Min range in the forward direction (sector 0 and its neighbors).
  double forwardClearance(const std::vector<double>& sector_min)
  {
    const int fwd = num_sectors_ / 2;
    double m = sector_min[fwd];
    m = std::min(m, sector_min[(fwd + 1) % num_sectors_]);
    m = std::min(m, sector_min[(fwd - 1 + num_sectors_) % num_sectors_]);
    return m;
  }

  // ------------------------------------------------------------------ timers
  void frontierCb(const ros::TimerEvent&)
  {
    if (state_ != EXPLORE || !map_received_)
      return;

    // Load hardcoded waypoints (world -> map frame) on the first cycle once the
    // map/tf is available. Runs before manual-goal handling so the preset
    // waypoints are already in manual_goals_ for this cycle.
    if (!preset_loaded_ && !preset_waypoints_.empty())
      preset_loaded_ = loadPresetWaypoints();

    // tracking 接管期间不推进航点、不超时、不切目标；RESUME 后 controlCb
    // 会把暂停点插回队首并强制重新规划。
    if (mission_paused_)
      return;

    const nav_msgs::OccupancyGrid& m = map_;
    const int w = m.info.width;
    const int h = m.info.height;
    const double res = m.info.resolution;
    const double ox = m.info.origin.position.x;
    const double oy = m.info.origin.position.y;

    // Robot cell in the SLAM map frame (via tf). The mavros pose is in the EKF
    // frame, which slam_toolbox rotates/offsets, so pose_x_/pose_y_ must NOT be
    // used directly against the map origin.
    double mx, my, myaw;
    if (!getMapPose(mx, my, myaw))
      return;
    const double rx = (mx - ox) / res;
    const double ry = (my - oy) / res;

    // --- Manual multi-point navigation (priority over frontier exploration).
    // Each rviz "2D Nav Goal" click is appended to a FIFO queue. The head goal
    // is turned into a global A* path in the SLAM map frame, which is then
    // string-pulled into a compact waypoint list; the control loop flies those
    // waypoints fast along straight segments (not exploration's cautious
    // cell-by-cell pure pursuit). The A* plan is refreshed when the goal
    // changes, every manual_replan_interval_, or when the controller gets
    // stuck against something new. On reach (or timeout) the head pops and the
    // next goal is planned. When the queue drains, autonomous frontier
    // exploration resumes.
    if (!manual_goals_.empty())
    {
      const ros::Time now = ros::Time::now();
      const double gx = manual_goals_.front().first;
      const double gy = manual_goals_.front().second;

      const bool goal_changed = !goal_set_ ||
          std::hypot(goal_x_ - gx, goal_y_ - gy) > 1e-6;
      const bool replan_due = replan_requested_ ||
          (now - last_manual_plan_).toSec() > manual_replan_interval_;

      if (goal_changed || replan_due)
      {
        if (goal_changed) setGoal(gx, gy, now);
        planPath(mx, my, myaw, gx, gy, plan_inflation_,
                 manual_unknown_as_obstacle_);  // global A* (inflated)
        // Far away (> transfer_dist_): fly straight ONLY if the direct line is
        // obstacle-free on the map; at 5.5 m the lidar only sees tall
        // obstacles, so a straight line is fast and safe across open ground.
        // But if the straight segment crosses baked buildings (the low walls
        // the lidar cannot see), fall back to the A* waypoints and go around.
        // Near: use the pruned A* waypoints for the final approach.
        const double dist_to_goal = std::hypot(gx - mx, gy - my);
        const bool line_clear = segmentClear(mx, my, gx, gy);
        // First leg from spawn: the map around the start is almost entirely
        // unknown, so a straight transfer leg is blind flight through whatever
        // buildings the lidar has not seen yet (drone2/5 slammed into house_3
        // / fast_food exactly this way). Only allow transfer once the drone
        // has reached at least one waypoint and the surroundings are mapped.
        const bool first_leg = (manual_waypoints_flown_ == 0);
        if (!first_leg && dist_to_goal > transfer_dist_ && line_clear)
        {
          transfer_mode_ = true;
          manual_wps_.clear();
          wp_idx_ = 0;
        }
        else
        {
          transfer_mode_ = false;
          simplifyPathToWaypoints();  // -> few straight legs
          rebaseWaypointIndex(mx, my);
        }
        replan_requested_ = false;
        last_manual_plan_ = now;
        publishPathMarker();
        ROS_INFO("[auto_explore] manual goal (%.1f, %.1f): A* %zu cells -> %zu waypoints (dist %.0f m)%s",
                 gx, gy, path_.size(), manual_wps_.size(), dist_to_goal,
                 transfer_mode_ ? " [TRANSFER]" : "");
        if (!manual_wps_.empty() || transfer_mode_)
        {
          // A* found a path (or the straight line is verified clear): remember
          // this pose for backtrack recovery.
          have_retreat_ = true;
          retreat_x_ = mx;
          retreat_y_ = my;
          last_astar_ok_ = now;
        }
      }

      // Waypoints exhausted => the drone sits on the (last waypoint =) goal
      // cell; allow up to wp_reach_tolerance_ so it is popped promptly.
      const bool reached = manual_wps_.empty()
          ? std::hypot(gx - mx, gy - my) <
                std::max(goal_tolerance_, wp_reach_tolerance_)
          : wp_idx_ >= manual_wps_.size();
      const bool timed_out =
          (now - goal_stamp_).toSec() > manual_goal_timeout_;

      if (reached || timed_out)
      {
        ROS_INFO("[auto_explore] manual goal (%.1f, %.1f) %s, %zu left",
                 gx, gy, reached ? "reached" : "timed out",
                 manual_goals_.size() - 1);
        manual_goals_.pop_front();
        manual_wps_.clear();
        wp_idx_ = 0;
        transfer_mode_ = false;
        goal_set_ = false;
        path_.clear();
        publishPathMarker();  // drop the A* line
        last_frontier_time_ = now;  // reset idle clock after manual phase
        ++manual_waypoints_flown_;  // first leg done -> descend to patrol altitude
      }
      return;  // manual goals handled; skip frontier detection this cycle
    }

    // --- Frontier clustering + information-gain exploration.
    // Collect all frontier cells (FREE adjacent to UNKNOWN), group adjacent
    // ones into clusters, and pick the cluster with the best
    // information/effort ratio: score = cluster_size / (dist + k). This
    // targets the LARGEST unexplored regions first (whole-area coverage) and
    // avoids creeping cell-by-cell in a nearby corner (the old
    // distance-only pick caused local thrashing). Multi-drone dedup: clusters
    // whose centroid is already claimed by another drone (peer goal markers)
    // are skipped so the six drones spread across different regions instead
    // of re-scanning the same one.
    std::vector<uint8_t> visited(static_cast<size_t>(w) * h, 0);
    struct Cluster { double cx, cy; int size; };
    std::vector<Cluster> clusters;
    const int di[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dj[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    for (int y = 1; y < h - 1; ++y)
    {
      for (int x = 1; x < w - 1; ++x)
      {
        const int idx = y * w + x;
        if (visited[idx] || m.data[idx] != 0) continue;  // FREE, unvisited
        bool frontier = false;
        for (int dy = -1; dy <= 1 && !frontier; ++dy)
        {
          const int yy = y + dy;
          if (yy < 0 || yy >= h) continue;
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (!(dx || dy)) continue;
            const int xx = x + dx;
            if (xx < 0 || xx >= w) continue;
            if (m.data[yy * w + xx] == -1)
            { frontier = true; break; }
          }
        }
        if (!frontier) continue;

        // Flood-fill this frontier cluster.
        double sx = 0.0, sy = 0.0;
        int cnt = 0;
        std::vector<int> stack;
        stack.push_back(idx);
        visited[idx] = 1;
        while (!stack.empty())
        {
          const int c = stack.back(); stack.pop_back();
          const int cx2 = c % w, cy2 = c / w;
          sx += cx2; sy += cy2; ++cnt;
          for (int k = 0; k < 8; ++k)
          {
            const int nx = cx2 + dj[k], ny = cy2 + di[k];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            const int ni = ny * w + nx;
            if (visited[ni] || m.data[ni] != 0) continue;
            bool nf = false;
            for (int dy = -1; dy <= 1 && !nf; ++dy)
            {
              const int yy = ny + dy;
              if (yy < 0 || yy >= h) continue;
              for (int dx = -1; dx <= 1; ++dx)
              {
                if (!(dx || dy)) continue;
                const int xx = nx + dx;
                if (xx < 0 || xx >= w) continue;
                if (m.data[yy * w + xx] == -1)
                { nf = true; break; }
              }
            }
            if (nf) { visited[ni] = 1; stack.push_back(ni); }
          }
        }
        if (cnt >= 3)  // ignore 1-2 cell noise frontiers
          clusters.push_back({sx / cnt, sy / cnt, cnt});
      }
    }

    int best_x = -1, best_y = -1;
    double best_gain = -1.0;
    for (const Cluster& cl : clusters)
    {
      // Skip clusters claimed by a peer (another drone's active frontier
      // goal within 8 m of this centroid).
      bool claimed = false;
      for (const PeerGoal& pg : peer_goals_)
      {
        const double dxm = (cl.cx * res + ox) - pg.x;
        const double dym = (cl.cy * res + oy) - pg.y;
        if (dxm * dxm + dym * dym < 8.0 * 8.0) { claimed = true; break; }
      }
      if (claimed) continue;
      const double d = std::hypot(cl.cx - rx, cl.cy - ry);
      // info gain = cluster size; effort = distance (+ 20 m so nearby small
      // clusters still beat far huge ones only when gain is really larger).
      const double gain = static_cast<double>(cl.size) / (d + 20.0);
      if (gain > best_gain)
      {
        best_gain = gain;
        best_x = static_cast<int>(cl.cx);
        best_y = static_cast<int>(cl.cy);
      }
    }

    const ros::Time now = ros::Time::now();
    if (best_x >= 0)
    {
      saw_frontier_ = true;
      last_frontier_time_ = now;
      const double fx = ox + (best_x + 0.5) * res;
      const double fy = oy + (best_y + 0.5) * res;
      // Reached = the drone is close to the CURRENT goal (not the candidate
      // frontier), otherwise a goal already under the drone is never cleared
      // and the drone spins in place pointing at it.
      const bool reached = goal_set_ &&
                           (std::hypot(goal_x_ - mx, goal_y_ - my) < goal_tolerance_);
      const bool timed_out = goal_set_ &&
                             (now - goal_stamp_).toSec() > goal_timeout_;
      if (!goal_set_ || reached || timed_out)
        setGoal(fx, fy, now);
    }
    else if (saw_frontier_ && (now - last_frontier_time_).toSec() > idle_timeout_)
    {
      ROS_INFO("[auto_explore] exploration complete (no frontier for %.1fs)",
               idle_timeout_);
      state_ = DONE;
      sendCmd("HOVER");
      saveMap();
    }

    // (Re)compute the global path to the current goal once per frontier cycle,
    // so map growth is folded into the plan at ~frontier_rate.
    if (goal_set_ && state_ == EXPLORE)
      planPath(mx, my, myaw, goal_x_, goal_y_);
  }

  void controlCb(const ros::TimerEvent&)
  {
    geometry_msgs::Twist cmd;

    // Publish own map-frame pose so peers can avoid collisions.
    {
      double mx, my, myaw;
      if (getMapPose(mx, my, myaw))
      {
        // Throttle pose broadcast to 5 Hz: enough for collision avoidance,
        // and keeps the swarm topic traffic (6 drones x 20 Hz would add load
        // that destabilises the fragile gzserver/libprotobuf under heavy use).
        const ros::Time now = ros::Time::now();
        if ((now - last_pose_pub_).toSec() >= 0.2)
        {
          last_pose_pub_ = now;
          geometry_msgs::PoseStamped p;
          p.header.stamp = now;
          p.header.frame_id = vehicle_;  // frame_id carries the vehicle name
          p.pose.position.x = mx;
          p.pose.position.y = my;
          p.pose.position.z = pose_z_;
          p.pose.orientation.w = 1.0;
          drone_pose_pub_.publish(p);
        }
      }
    }

    // 处理 tracking 节点的 PAUSE/RESUME 请求。需要 map 位姿，所以放在这里
    // 而不是订阅回调里（tf 查询放到控制循环线程）。
    if (pause_requested_ || resume_requested_)
    {
      double mx, my, myaw;
      if (getMapPose(mx, my, myaw))
      {
        if (pause_requested_ && !mission_paused_)
        {
          pause_x_ = mx;
          pause_y_ = my;
          have_pause_pose_ = true;
          mission_paused_ = true;
          ROS_INFO("[auto_explore] mission PAUSED at (%.1f, %.1f), "
                   "holding waypoint state", mx, my);
        }
        else if (resume_requested_ && mission_paused_)
        {
          if (have_pause_pose_)
          {
            // 把暂停点插回队首：tracking 结束后先飞回刚才的位置，
            // 再继续原来的航点序列。
            manual_goals_.push_front(std::make_pair(pause_x_, pause_y_));
            manual_wps_.clear();
            wp_idx_ = 0;
            goal_set_ = false;
            path_.clear();
            transfer_mode_ = false;
            ROS_INFO("[auto_explore] mission RESUMED, returning to pause pose "
                     "(%.1f, %.1f)", pause_x_, pause_y_);
            have_pause_pose_ = false;
          }
          mission_paused_ = false;
        }
        pause_requested_ = false;
        resume_requested_ = false;
      }
    }

    switch (state_)
    {
      case TAKEOFF:
      {
        const ros::Time now = ros::Time::now();
        if ((now - last_cmd_sent_).toSec() > 1.0)
        {
          sendCmd("OFFBOARD");
          sendCmd("ARM");
          last_cmd_sent_ = now;
        }

        // 起飞阶段 GPS/位姿水平位置保持：
        // 记录起飞原点，爬升过程中若 EKF 位置发生水平漂移，
        // 发布反向水平速度把无人机拉回原点，防止 6 机在 3m 间距下互相碰撞。
        if (!takeoff_origin_set_ && pose_z_ > 0.1)
        {
          takeoff_origin_x_ = pose_x_;
          takeoff_origin_y_ = pose_y_;
          takeoff_origin_set_ = true;
          ROS_INFO("[auto_explore] takeoff position hold enabled, origin (%.2f, %.2f)",
                   takeoff_origin_x_, takeoff_origin_y_);
        }
        if (takeoff_origin_set_)
        {
          const double err_x = pose_x_ - takeoff_origin_x_;  // ENU east 误差
          const double err_y = pose_y_ - takeoff_origin_y_;  // ENU north 误差
          const double v_east = -takeoff_pos_kp_ * err_x;
          const double v_north = -takeoff_pos_kp_ * err_y;
          const double cy = std::cos(pose_yaw_);
          const double sy = std::sin(pose_yaw_);
          // ENU -> FLU（起飞阶段 yaw 变化很小）
          cmd.linear.x = clamp(sy * v_east + cy * v_north, takeoff_max_horiz_);
          cmd.linear.y = clamp(-cy * v_east + sy * v_north, takeoff_max_horiz_);
        }

        const double zerr = target_altitude_ - pose_z_;
        cmd.linear.z = clamp(k_z_ * zerr, max_lin_z_);
        publishSmooth(cmd);

        if (std::fabs(zerr) < 0.3)
        {
          state_ = EXPLORE;
          last_frontier_time_ = ros::Time::now();
          ROS_INFO("[auto_explore] altitude %.1f reached, exploring", pose_z_);
        }
        break;
      }

      case EXPLORE:
      {
        // altitude hold
        cmd.linear.z = clamp(k_z_ * (target_altitude_ - pose_z_), max_lin_z_);

        // tracking 接管期间：explore_node 不输出导航速度，mux 由 tracking
        // 控制；RESUME 后下一周期恢复正常航点飞行。
        if (mission_paused_)
        {
          publishSmooth(cmd);
          break;
        }

        // Manual goals queued: fly the A* waypoint path (fast straight
        // segments) instead of the exploration pure-pursuit controller.
        if (!manual_goals_.empty())
        {
          double mx, my, myaw;
          if (!getMapPose(mx, my, myaw))
          {
            publishSmooth(cmd);  // hover until tf is available
            break;
          }
          if (goal_set_)
          {
            // Transfer altitude: climb above the buildings for the first leg
            // (spawn -> first waypoint) and for A* fallback legs, so the drone
            // flies straight over unmapped/building areas instead of creeping
            // blind through them. Descend to the patrol altitude within
            // descend_dist_ of the goal.
            const double dist = std::hypot(goal_x_ - mx, goal_y_ - my);
            const bool transfer =
                (manual_waypoints_flown_ == 0 && dist > descend_dist_) ||
                (manual_wps_.empty() && dist > descend_dist_);
            const double alt = transfer ? transfer_altitude_ : target_altitude_;
            cmd.linear.z = clamp(k_z_ * (alt - pose_z_), max_lin_z_);
            manualFlightControl(cmd, mx, my, myaw);
            publishSmooth(cmd);
          }
          else
          {
            cmd.angular.z = 0.3;  // queued but not planned yet: yaw in place
            publishSmooth(cmd);
          }
          break;
        }

        if (!goal_set_ || !scan_received_)
        {
          // wait for a goal / first scan: yaw slowly in place
          cmd.angular.z = 0.3;
          publishSmooth(cmd);
          break;
        }

        double mx, my, myaw;
        if (!getMapPose(mx, my, myaw))
        {
          publishSmooth(cmd);  // hover until tf is available
          break;
        }
        // Pure pursuit on the A* path: aim at the first path point at least
        // lookahead_dist_ ahead of the drone, falling back to the raw goal when
        // no path is available (or the drone is within a lookahead of the end).
        double target_x = goal_x_, target_y = goal_y_;
        if (!path_.empty())
        {
          const double ox = map_.info.origin.position.x;
          const double oy = map_.info.origin.position.y;
          const double res = map_.info.resolution;
          bool found_target = false;
          for (const astar::Cell& c : path_)
          {
            const double wx = ox + (c.x + 0.5) * res;
            const double wy = oy + (c.y + 0.5) * res;
            if (std::hypot(wx - mx, wy - my) >= lookahead_dist_)
            {
              target_x = wx;
              target_y = wy;
              found_target = true;
              break;
            }
          }
          if (!found_target)
          {
            const astar::Cell& c = path_.back();
            target_x = ox + (c.x + 0.5) * res;
            target_y = oy + (c.y + 0.5) * res;
          }
        }
        // MPPI local planner: replaces the old sector-based heading/speed
        // selection for frontier exploration. The global A* path is still
        // used only to pick the lookahead target point (target_x, target_y).
        applyMppiControl(mx, my, myaw, target_x, target_y, cmd);

        cmd.linear.x = rampLinX(cmd.linear.x);
        avoidPeers(cmd, mx, my, myaw);  // multi-drone collision avoidance
        publishSmooth(cmd);
        break;
      }

      case DONE:
      {
        publishSmooth(cmd);  // all-zero => hover in place
        break;
      }
    }
  }

  void saveMap()
  {
    ROS_INFO("[auto_explore] saving map to %s", map_save_path_.c_str());
    const std::string cmd = "rosrun map_server map_saver -f " + map_save_path_ +
                            " map:=" + map_topic_;
    const int ret = std::system(cmd.c_str());
    if (ret != 0)
      ROS_WARN("[auto_explore] map_saver returned %d", ret);
    else
      ROS_INFO("[auto_explore] map saved to %s.pgm / %s.yaml",
               map_save_path_.c_str(), map_save_path_.c_str());
  }

  // ------------------------------------------------------------------ members
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  State state_;
  bool goal_set_;
  double goal_x_, goal_y_;
  ros::Time goal_stamp_;
  double pose_x_, pose_y_, pose_z_, pose_yaw_;
  double takeoff_origin_x_, takeoff_origin_y_;
  bool takeoff_origin_set_;
  double takeoff_pos_kp_, takeoff_max_horiz_;
  bool map_received_, scan_received_;
  bool saw_frontier_;
  bool manual_unknown_as_obstacle_;
  sensor_msgs::LaserScan scan_;
  nav_msgs::OccupancyGrid map_;

  ros::Subscriber pose_sub_, scan_sub_, map_sub_, manual_goal_sub_, peer_goal_sub_, drone_pose_sub_;
  ros::Subscriber mission_control_sub_;
  bool mission_paused_ = false;
  bool pause_requested_ = false;
  bool resume_requested_ = false;
  double pause_x_ = 0.0, pause_y_ = 0.0;
  bool have_pause_pose_ = false;
  ros::Publisher cmd_vel_pub_, cmd_pub_, marker_pub_, explore_goal_pub_, drone_pose_pub_;
  ros::Timer control_timer_, frontier_timer_;
  ros::Time takeoff_start_, last_cmd_sent_, last_frontier_time_;

  // Peers' shared exploration goals (multi-drone dedup).
  struct PeerGoal
  {
    std::string ns;
    double x, y;
    ros::Time stamp;
  };
  std::vector<PeerGoal> peer_goals_;

  // Peers' map-frame poses (multi-drone collision avoidance).
  struct PeerPose
  {
    std::string ns;
    double x, y;
    ros::Time stamp;
  };
  std::vector<PeerPose> peer_poses_;
  double avoid_dist_;
  mppi::Planner mppi_;
  double spawn_x_ = 0.0, spawn_y_ = 0.0;
  bool have_spawn_ = false;
  ros::Time last_pose_pub_;

  std::string pose_topic_, scan_topic_, map_topic_, cmd_vel_topic_, cmd_topic_;
  std::string goal_topic_;
  std::string map_frame_, base_frame_, odom_frame_, map_save_path_;
  std::string vehicle_;

  double target_altitude_, min_lin_x_, max_lin_x_, max_lin_z_, max_yaw_rate_;
  double k_lin_, k_yaw_, k_z_;
  double accel_limit_;
  double last_lin_x_ = 0.0;
  // Output smoothing state
  ros::Time last_smooth_time_;
  double smooth_alpha_ = 0.3;
  double smooth_rate_xyz_ = 1.2;
  double smooth_rate_yaw_ = 1.0;
  double smooth_x_ = 0.0, smooth_y_ = 0.0, smooth_z_ = 0.0, smooth_yaw_ = 0.0;
  double last_smooth_x_ = 0.0, last_smooth_y_ = 0.0, last_smooth_z_ = 0.0, last_smooth_yaw_ = 0.0;
  double goal_tolerance_, goal_timeout_, safe_distance_, robot_radius_;
  double frontier_rate_, control_rate_, idle_timeout_, takeoff_timeout_;
  int num_sectors_;
  double lookahead_dist_;
  int occupied_thresh_;
  double manual_goal_timeout_;
  double manual_max_yaw_rate_, wp_reach_tolerance_, manual_brake_dist_;
  double manual_replan_interval_, plan_inflation_;
  double plan_clear_radius_;
  double transfer_altitude_, descend_dist_, recovery_timeout_, backtrack_dist_, transfer_dist_;
  int manual_waypoints_flown_ = 0;
  // Backtrack recovery: the last pose where A* found a path. If A* keeps
  // failing (drone trapped in a building cluster), fly back here instead of
  // blind-creeping toward the goal, until the planner recovers.
  double retreat_x_ = 0.0, retreat_y_ = 0.0;
  bool have_retreat_ = false;
  ros::Time last_astar_ok_;
  std::vector<astar::Cell> path_;
  std::deque<std::pair<double, double>> manual_goals_;
  std::vector<std::pair<double, double>> manual_wps_;  // pruned A* waypoints (map frame)
  size_t wp_idx_ = 0;
  bool transfer_mode_ = false;  // straight-line transfer leg (verified clear)
  std::vector<std::pair<double, double>> preset_waypoints_;
  bool preset_loaded_ = false;
  ros::Time last_manual_plan_;
  bool replan_requested_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore_node");
  ExploreNode node;
  ros::spin();
  return 0;
}
