#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <limits>
#include <string>

// Cleans the drone's 2D scan before SLAM / avoidance:
//
//  1. Self-occlusion cut: the lidar sits on top of a hexacopter whose
//     arms/propellers cross the scan plane (especially when the airframe
//     pitches/rolls), so near-range self-hits get written into the map as
//     obstacles and trace a "trail". Any reading closer than `cut_radius` is
//     rewritten to +inf.
//
//  2. Tilt compensation: a 2D lidar mounted on the body tilts with the drone
//     (measured ~10-18 deg pitch while cruising at speed). A tilted scan
//     reports every wall hit 1/cos(tilt) too far and catches roof edges /
//     ground rings, which smears the map into scattered dots. Each ray is
//     reprojected onto the horizontal plane using the vehicle's roll/pitch:
//     range' = range * |horizontal component of the (tilted) ray direction|.
//
//  3. Ground-hit removal: a downward ray whose vertical drop equals the
//     lidar's height above the ground is a ground hit (fake obstacle ring).
//     It is rewritten to +inf.
//
// karto's OccupancyGrid::AddScan skips readings >= maxRange (and NaN) — it
// neither marks them occupied nor clears space through them — which is exactly
// the semantics we want. +inf is also skipped by explore_node's sector
// builder, so avoidance stops seeing its own arms / the ground as obstacles.
class ScanSelfFilter
{
public:
  ScanSelfFilter()
    : pnh_("~"),
      roll_(0.0), pitch_(0.0), pose_z_(0.0), pose_valid_(false)
  {
    pnh_.param<std::string>("scan_topic", scan_topic_, "/typhoon_h480_0/scan");
    pnh_.param<std::string>("scan_filtered_topic", scan_filtered_topic_,
                            "/typhoon_h480_0/scan_filtered");
    pnh_.param<std::string>("pose_topic", pose_topic_,
                            "/typhoon_h480_0/mavros/local_position/pose");
    pnh_.param<double>("cut_radius", cut_radius_, 0.7);
    pnh_.param<double>("laser_z_offset", laser_z_offset_, 0.6);   // lidar above base_link
    pnh_.param<double>("ground_alt_offset", ground_alt_offset_, 1.0);  // EKF origin 高于地面
    pnh_.param<double>("ground_margin", ground_margin_, 0.8);

    scan_sub_ = nh_.subscribe(scan_topic_, 1, &ScanSelfFilter::scanCb, this);
    pose_sub_ = nh_.subscribe(pose_topic_, 1, &ScanSelfFilter::poseCb, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_filtered_topic_, 1);
    ROS_INFO("[scan_self_filter] %s -> %s (cut=%.2f m, tilt-comp on, ground-margin %.2f m)",
             scan_topic_.c_str(), scan_filtered_topic_.c_str(),
             cut_radius_, ground_margin_);
  }

private:
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                      msg->pose.orientation.z, msg->pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    roll_ = r;
    pitch_ = p;
    pose_z_ = msg->pose.position.z;
    pose_valid_ = true;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    sensor_msgs::LaserScan out = *msg;  // preserve header/frame/range_min/max
    const float inf = std::numeric_limits<float>::infinity();
    const float rmax = msg->range_max;
    const double sin_r = std::sin(roll_), cos_r = std::cos(roll_);
    const double sin_p = std::sin(pitch_), cos_p = std::cos(pitch_);
    // 激光离地高度 = EKF z + 安装偏移 + (EKF 原点高于地面的量)
    const double laser_h = pose_z_ + laser_z_offset_ + ground_alt_offset_;

    for (size_t i = 0; i < out.ranges.size(); ++i)
    {
      float r = out.ranges[i];
      if (r < cut_radius_)          // self-hit: cut
      {
        out.ranges[i] = inf;
        continue;
      }
      if (!(r >= msg->range_min && r <= rmax)) continue;  // out of range / NaN

      const double a = msg->angle_min + i * msg->angle_increment;
      const double ca = std::cos(a), sa = std::sin(a);
      // 机体水平射线 (ca,sa,0) 先绕 X 滚转 roll，再绕 Y 俯仰 pitch，
      // 取 Z 分量（向上为正）与水平分量。
      const double d2z = -ca * sin_p + sa * sin_r * cos_p;
      if (pose_valid_ && d2z < 0.0 &&
          std::fabs(r * (-d2z) - laser_h) < ground_margin_)
      {
        out.ranges[i] = inf;  // ground hit: drop
        continue;
      }
      const double hmag = std::sqrt(std::max(0.0, 1.0 - d2z * d2z));
      out.ranges[i] = static_cast<float>(r * hmag);  // tilt-compensated range
    }
    scan_pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_, pose_sub_;
  ros::Publisher scan_pub_;
  std::string scan_topic_, scan_filtered_topic_, pose_topic_;
  double cut_radius_, laser_z_offset_, ground_alt_offset_, ground_margin_;
  double roll_, pitch_, pose_z_;
  bool pose_valid_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_self_filter");
  ScanSelfFilter node;
  ros::spin();
  return 0;
}
