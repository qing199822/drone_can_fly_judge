// src/multi_mission_launcher.cpp

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <string>
#include <fstream>
#include <streambuf>
#include "task_manager/mission_manager.h"
#include <json/json.h>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "用法錯誤!" << std::endl;
        std::cerr << "用法: rosrun task_manager multi_mission_launcher <mission_file_path> <vehicle_id_1> [vehicle_id_2] ..." << std::endl;
        std::cerr << "示例: rosrun task_manager multi_mission_launcher ~/mission.json typhoon_h480_0 iris_0" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "multi_mission_launcher");
    ros::NodeHandle nh;

    std::string mission_file_path = argv[1];
    std::vector<std::string> target_vehicle_ids;
    for (int i = 2; i < argc; ++i) {
        target_vehicle_ids.push_back(argv[i]);
    }

    // 讀取並解析 JSON 文件
    std::ifstream mission_file(mission_file_path);
    if (!mission_file.is_open()) {
        ROS_FATAL_STREAM("[Launcher] 無法打開任務文件: " << mission_file_path);
        return 1;
    }

    Json::Value all_missions;
    Json::Reader reader;
    if (!reader.parse(mission_file, all_missions)) {
        ROS_FATAL_STREAM("[Launcher] 解析JSON文件失敗: " << reader.getFormattedErrorMessages());
        return 1;
    }

    std::vector<std::thread> threads;

    for (const auto& vehicle_id : target_vehicle_ids) {
        ROS_INFO("[Launcher] 正在為 %s 準備任務...", vehicle_id.c_str());

        bool mission_found = false;
        for (const auto& mission_data : all_missions) {
            if (mission_data["vehicle_id"].asString() == vehicle_id) {
                mission_found = true;
                std::vector<Waypoint> waypoints;
                const Json::Value& wp_json = mission_data["waypoints"];
                for (const auto& point : wp_json) {
                    waypoints.push_back({point["x"].asDouble(), point["y"].asDouble(), point["z"].asDouble()});
                }
                
                // 創建 MissionManager 實例並啟動線程
                auto manager = std::make_shared<MissionManager>(vehicle_id, waypoints);
                threads.emplace_back(&MissionManager::run_mission, manager);

                ROS_INFO("[Launcher] 已為 %s 啟動任務線程。", vehicle_id.c_str());
                break;
            }
        }

        if (!mission_found) {
            ROS_WARN("[Launcher] 在文件 %s 中未找到無人機ID '%s' 的任務，已跳過。", mission_file_path.c_str(), vehicle_id.c_str());
        }
    }

    for (auto& th : threads) {
        if (th.joinable()) {
            th.join();
        }
    }

    ROS_INFO("[Launcher] 所有無人機的任務均已完成。主程序退出。");

    return 0;
}
