#include <ros/ros.h>
#include "fly/fly_takeoff.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "confident_takeoff_node");
    
    if (argc != 4) {
        ROS_ERROR("Usage: fly_takeoff <drone_name> <drone_quantity> <target_altitude>");
        return 1;
    }
    
    std::string drone_name = argv[1];
    int drone_quantity = std::stoi(argv[2]);
    double target_altitude = std::stod(argv[3]);
    
    ROS_INFO("Starting final revised takeoff mission: model=%s, quantity=%d, target altitude=%.2f meters", 
             drone_name.c_str(), drone_quantity, target_altitude);
    
    try {
        fly::ConfidentTakeoff controller(drone_name, drone_quantity, target_altitude);
        controller.run();
        ROS_INFO("Autonomous takeoff script has exited.");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception occurred: %s", e.what());
        return 1;
    }
    
    return 0;
}
