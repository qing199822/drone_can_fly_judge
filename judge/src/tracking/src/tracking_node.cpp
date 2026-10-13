#include <ros/ros.h>
#include <string>
#include <mutex> // For std::mutex and std::lock_guard

// ROS Message Types
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// All the custom module headers
#include "tracking/state_machine.h"
#include "tracking/controller.h"
#include "tracking/service_manager.h"

/**
 * @class TrackingNode
 * @brief The main node class that orchestrates the entire tracking process.
 *
 * This class initializes all components (state machine, controller, services),
 * subscribes to necessary ROS topics, and runs the main control loop.
 */
class TrackingNode {
public:
    /**
     * @brief Constructor for the main tracking node.
     * @param nh ROS NodeHandle.
     * @param vehicle_type The type of the vehicle.
     * @param vehicle_id The ID of the vehicle.
     */
    TrackingNode(ros::NodeHandle& nh, const std::string& vehicle_type, const std::string& vehicle_id)
        : nh_(nh), vehicle_type_(vehicle_type), vehicle_id_(vehicle_id), pose_received_(false)
    {
        nh_.param("detection_timeout", detection_timeout_, 2.0);
        ROS_INFO("Initializing HumanTrackingNode for %s_%s...", vehicle_type_.c_str(), vehicle_id_.c_str());

        // Instantiate all the components
        // Note the dependency injection: StateMachine uses the controller and services.
        controller_ = std::make_unique<TrackingController>(nh_);
        services_ = std::make_unique<ServiceManager>(nh_, vehicle_type_, vehicle_id_);
        state_machine_ = std::make_unique<TrackingStateMachine>(nh_, vehicle_type_, vehicle_id_, *controller_, *services_);

        // Set up ROS subscribers
        setupSubscribers();
    }

    /**
     * @brief Runs the main loop of the node.
     */
    void run() {
        waitForPose();
        ROS_INFO("[%s_%s Tracker] Initialization complete, starting main loop.",
                 vehicle_type_.c_str(), vehicle_id_.c_str());

        ros::Rate rate(30);
        while (ros::ok()) {
            // Create thread-safe copies of the data received from callbacks
            TargetMap visible_targets_copy;
            geometry_msgs::Pose pose_copy;
            geometry_msgs::Twist velocity_copy;
            double height_copy;

            { // Scoped lock to minimize contention
                std::lock_guard<std::mutex> lock(data_mutex_);
                // 如果 YOLO 长时间没有发布新检测（例如节点退出/相机中断），
                // 清空当前可见目标，避免 tracking 拿着最后一帧旧目标永远追下去。
                if (!last_detection_time_.is_zero() &&
                    (ros::Time::now() - last_detection_time_).toSec() > detection_timeout_) {
                    current_visible_targets_.clear();
                }
                visible_targets_copy = current_visible_targets_;
                pose_copy = current_pose_;
                velocity_copy = current_velocity_body_;
                height_copy = height_;
            }

            // Update the state machine with the latest data
            state_machine_->update(visible_targets_copy, height_copy, pose_copy, velocity_copy);

            // Process callbacks
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    /**
     * @brief Sets up all ROS topic subscribers.
     */
    void setupSubscribers() {
        std::string ns = "/" + vehicle_type_ + "_" + vehicle_id_;

        darknet_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>(
            ns + "/yolo11n/bounding_boxes", 1, &TrackingNode::darknetCallback, this);

        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            ns + "/mavros/local_position/pose", 1, &TrackingNode::poseCallback, this);

        velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
            ns + "/mavros/local_position/velocity_body", 1, &TrackingNode::velocityCallback, this);
    }

    /**
     * @brief Blocks execution until the first pose message is received.
     */
    void waitForPose() {
        ros::Rate rate(30);
        while (!pose_received_ && ros::ok()) {
            ROS_INFO_THROTTLE(2, "[%s_%s Tracker] Waiting for drone pose information...",
                              vehicle_type_.c_str(), vehicle_id_.c_str());
            ros::spinOnce(); // Needed to process callbacks
            rate.sleep();
        }
    }

    // --- Callback Functions ---

    void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_detection_time_ = ros::Time::now();
        current_visible_targets_.clear();
        for (const auto& target : msg->bounding_boxes) {
            // Assuming TARGET_IDS are handled by the state machine's request logic
            current_visible_targets_[target.Class] = target;
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pose_ = msg->pose;
        height_ = msg->pose.position.z;
        if (!pose_received_) {
            pose_received_ = true;
        }
    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_velocity_body_ = msg->twist;
    }

    // --- ROS and Core Components ---
    ros::NodeHandle& nh_;
    std::string vehicle_type_;
    std::string vehicle_id_;

    std::unique_ptr<TrackingController> controller_;
    std::unique_ptr<ServiceManager> services_;
    std::unique_ptr<TrackingStateMachine> state_machine_;

    // --- ROS Subscribers ---
    ros::Subscriber darknet_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;

    // --- Shared Data & Synchronization ---
    std::mutex data_mutex_;
    TargetMap current_visible_targets_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_velocity_body_;
    double height_ = 0.0;
    bool pose_received_ = false;
    ros::Time last_detection_time_;
    double detection_timeout_ = 2.0;
};


int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: rosrun tracking tracking_node <vehicle_type> <vehicle_id>" << std::endl;
        return 1;
    }

    std::string vehicle_type = argv[1];
    std::string vehicle_id = argv[2];
    std::string node_name = "yolo_human_tracking_" + vehicle_type + "_" + vehicle_id;

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~"); // Use private node handle to read parameters

    try {
        TrackingNode node(nh, vehicle_type, vehicle_id);
        node.run();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("An unhandled exception occurred: " << e.what());
        return 1;
    }

    return 0;
}