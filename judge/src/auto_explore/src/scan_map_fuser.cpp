#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// Multi-robot 2D occupancy fuser.
//
// Subscribes to every drone's filtered 2D scan (/<vehicle>_<id>/scan_filtered),
// looks up each drone's pose in the shared map frame through tf
// (map -> odom_<id> static spawn offsets + odom_<id> -> base_link_<id> from the
// per-drone odom_broadcaster), and ray-casts every ray into ONE global
// OccupancyGrid published as /map (frame "map").
//
// Why this instead of N slam_toolbox instances: in simulation the mavros EKF
// odom is already globally consistent (no drift), so scan matching buys
// nothing — fusing all lidars directly into a shared world-frame grid is exact,
// cheap, and gives the single merged map the user asked for. It also removes
// the map<->odom rotation mess entirely (map frame == world frame).
//
// Per-drone static transforms map -> odom_<id> are published here (the spawn
// offsets are known constants).
class ScanMapFuser
{
public:
  ScanMapFuser()
    : nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
  {
    nh_.param<int>("num_drones", num_drones_, 6);
    nh_.param<std::string>("vehicle_type", vehicle_type_, "typhoon_h480");
    nh_.param<std::string>("map_frame", map_frame_, "map");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_.param<double>("resolution", resolution_, 0.1);
    nh_.param<double>("map_update_rate", map_update_rate_, 5.0);
    nh_.param<int>("occupied_thresh", occupied_thresh_, 50);
    nh_.param<double>("max_range", max_range_, 20.0);
    // ===== 出生点布局（环境适配层）=====
    // map -> odom_<id> 的平移量就是该机出生点在世界系中的位置。默认 = 本地调试
    // 布局；比赛用官方 robocup.launch 时只需把 spawn_origin_x 覆盖为 0。
    // 本节点靠这几个参数与 pose_init / area_mission_node 保持一致。
    nh_.param<double>("spawn_origin_x", spawn_origin_x_, -17.0);
    nh_.param<double>("spawn_origin_y", spawn_origin_y_, -3.0);
    nh_.param<double>("spawn_stride_x", spawn_stride_x_, 3.0);
    nh_.param<double>("spawn_stride_y", spawn_stride_y_, 3.0);

    // Fixed map covering the base0 road network (world frame) plus margin.
    origin_x_ = -90.0;
    origin_y_ = -60.0;
    width_ = static_cast<int>((140.0 - origin_x_) / resolution_);   // 2300
    height_ = static_cast<int>((60.0 - origin_y_) / resolution_);   // 1200
    grid_.assign(width_ * height_, -1);  // unknown

    // Per-drone scan subscriptions.
    for (int i = 0; i < num_drones_; ++i)
    {
      const std::string vehicle = vehicle_type_ + "_" + std::to_string(i);
      const std::string topic = "/" + vehicle + "/scan_filtered";
      ros::Subscriber sub = nh_.subscribe<sensor_msgs::LaserScan>(topic, 1,
          boost::bind(&ScanMapFuser::scanCb, this, _1, i));
      scan_subs_.push_back(sub);
      scans_.push_back(sensor_msgs::LaserScan());
      fresh_.push_back(false);
      frame_ids_.push_back(base_frame_ + "_" + std::to_string(i));
    }

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    tf_static_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);

    // Static map -> odom_<id> spawn transforms are published from updateCb
    // (after ros::spin starts) via the LATCHED /tf_static publisher, exactly
    // once. Publishing from the constructor before spin deadlocks
    // (futex wait) under this node's tf_buffer; the latched publisher makes
    // late subscribers (explore_node etc.) receive the transforms anyway.

    update_timer_ = nh_.createTimer(ros::Duration(1.0 / map_update_rate_),
                                    &ScanMapFuser::updateCb, this);
    ROS_INFO("[scan_map_fuser] %d drones -> /map (%dx%d @ %.2f m)",
             num_drones_, width_, height_, resolution_);
  }

private:
  // Publish the static spawn transforms once via a LATCHED /tf_static
  // publisher. Re-broadcasting from a timer callback with
  // tf2_ros::StaticTransformBroadcaster deadlocks (futex wait) under the
  // fuser's own tf_buffer + updateCb loop, so we publish exactly once with a
  // plain latched publisher — late subscribers still receive it.
  void publishStaticTransforms(const ros::TimerEvent& = ros::TimerEvent())
  {
    tf2_msgs::TFMessage msg;
    for (int i = 0; i < num_drones_; ++i)
    {
      const double sx = spawn_origin_x_ + spawn_stride_x_ * (i % 2);
      const double sy = spawn_origin_y_ + spawn_stride_y_ * (i / 2);
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = ros::Time::now();
      ts.header.frame_id = map_frame_;
      ts.child_frame_id = "odom_" + std::to_string(i);
      ts.transform.translation.x = sx;
      ts.transform.translation.y = sy;
      ts.transform.translation.z = 0.0;
      ts.transform.rotation.w = 1.0;
      msg.transforms.push_back(ts);
    }
    ROS_INFO("[scan_map_fuser] publishing %zu static transforms to /tf_static",
             msg.transforms.size());
    tf_static_pub_.publish(msg);
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg, int id)
  {
    scans_[id] = *msg;
    fresh_[id] = true;
  }

  // Bresenham line from (x0,y0) to (x1,y1) in grid cells; clear each cell to
  // free (0). FREE ALWAYS WINS: a ray that passes through a cell proves it is
  // free, so transient obstacles (other drones' bodies, ground-hit artifacts,
  // one-off bad scans) get erased. Real walls keep re-occupying the cell every
  // scan (no ray can pass through a wall), so they persist.
  void clearLine(int x0, int y0, int x1, int y1)
  {
    const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;
    while (true)
    {
      if (x >= 0 && x < width_ && y >= 0 && y < height_)
        grid_[y * width_ + x] = 0;
      if (x == x1 && y == y1) break;
      const int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  void updateCb(const ros::TimerEvent&)
  {
    if (!tf_static_sent_)
    {
      publishStaticTransforms();  // once, from the spin thread
      tf_static_sent_ = true;
    }

    for (int i = 0; i < num_drones_; ++i)
    {
      if (!fresh_[i]) continue;
      const sensor_msgs::LaserScan& sc = scans_[i];

      // Pose of this drone in the shared map frame.
      double mx, my, myaw;
      try
      {
        geometry_msgs::TransformStamped ts =
            tf_buffer_.lookupTransform(map_frame_, frame_ids_[i], ros::Time(0));
        mx = ts.transform.translation.x;
        my = ts.transform.translation.y;
        tf2::Quaternion q(ts.transform.rotation.x, ts.transform.rotation.y,
                          ts.transform.rotation.z, ts.transform.rotation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, myaw);
      }
      catch (const tf2::TransformException&)
      {
        continue;  // drone not ready yet
      }

      const double rmax = (max_range_ > 0.0) ? max_range_ : sc.range_max;
      const int rx = static_cast<int>(std::floor((mx - origin_x_) / resolution_));
      const int ry = static_cast<int>(std::floor((my - origin_y_) / resolution_));

      for (size_t k = 0; k < sc.ranges.size(); ++k)
      {
        const float r = sc.ranges[k];
        // 恢复到原始逻辑：只有有效回波才 clear + mark。
        // 之前尝试对无回波/过滤射线也 clear，导致地面/自身过滤方向上
        // 真实障碍物被误清，地图被“擦掉”。因此这里仍跳过无效射线。
        if (!(r >= sc.range_min && r <= sc.range_max) || !std::isfinite(r))
          continue;
        const double a = sc.angle_min + k * sc.angle_increment + myaw;
        const double ex = mx + r * std::cos(a);
        const double ey = my + r * std::sin(a);
        const int ex_cell = static_cast<int>(std::floor((ex - origin_x_) / resolution_));
        const int ey_cell = static_cast<int>(std::floor((ey - origin_y_) / resolution_));

        clearLine(rx, ry, ex_cell, ey_cell);
        if (r < rmax - 0.05 && ex_cell >= 0 && ex_cell < width_ &&
            ey_cell >= 0 && ey_cell < height_)
          grid_[ey_cell * width_ + ex_cell] = 100;  // occupied hit
      }
      fresh_[i] = false;
    }

    nav_msgs::OccupancyGrid out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = map_frame_;
    out.info.resolution = resolution_;
    out.info.width = width_;
    out.info.height = height_;
    out.info.origin.position.x = origin_x_;
    out.info.origin.position.y = origin_y_;
    out.info.origin.orientation.w = 1.0;
    out.data.resize(grid_.size());
    for (size_t i = 0; i < grid_.size(); ++i)
      out.data[i] = grid_[i];
    map_pub_.publish(out);
  }

  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  int num_drones_;
  std::string vehicle_type_, map_frame_, base_frame_;
  double resolution_, map_update_rate_, max_range_;
  // 出生点布局（世界系），默认 = 本地调试布局；比赛环境用参数覆盖。
  double spawn_origin_x_ = -17.0, spawn_origin_y_ = -3.0;
  double spawn_stride_x_ = 3.0, spawn_stride_y_ = 3.0;
  int occupied_thresh_;
  double origin_x_, origin_y_;
  int width_, height_;
  std::vector<int8_t> grid_;

  std::vector<ros::Subscriber> scan_subs_;
  std::vector<sensor_msgs::LaserScan> scans_;
  std::vector<bool> fresh_;
  std::vector<std::string> frame_ids_;
  ros::Publisher map_pub_, tf_static_pub_;
  ros::Timer update_timer_;
  bool tf_static_sent_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_map_fuser");
  ScanMapFuser node;
  ros::spin();
  return 0;
}
