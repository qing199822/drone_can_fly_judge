#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

// Republishes the drone's mavros local_position/pose as an odom->base_link tf.
// slam_toolbox then publishes map->odom (approx identity here, since the EKF
// pose is globally accurate in simulation). Frame names are parameterized so
// the package is reusable across vehicles.
class OdomBroadcaster
{
public:
  OdomBroadcaster()
    : pnh_("~")
  {
    pnh_.param<std::string>("pose_topic", pose_topic_, "/typhoon_h480_0/mavros/local_position/pose");
    pnh_.param<std::string>("parent_frame", parent_frame_, "odom");
    pnh_.param<std::string>("child_frame", child_frame_, "base_link");

    pose_sub_ = nh_.subscribe(pose_topic_, 1, &OdomBroadcaster::poseCb, this);
    ROS_INFO("[odom_broadcaster] %s -> %s from %s",
             parent_frame_.c_str(), child_frame_.c_str(), pose_topic_.c_str());
  }

private:
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = child_frame_;
    tf.transform.translation.x = msg->pose.position.x;
    tf.transform.translation.y = msg->pose.position.y;
    tf.transform.translation.z = msg->pose.position.z;
    tf.transform.rotation = msg->pose.orientation;
    br_.sendTransform(tf);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pose_sub_;
  tf2_ros::TransformBroadcaster br_;
  std::string pose_topic_, parent_frame_, child_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_broadcaster");
  OdomBroadcaster node;
  ros::spin();
  return 0;
}
