cmake_minimum_required(VERSION 3.0.2)
project(auto_explore)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  topic_tools
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs topic_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# odom_broadcaster: mavros local_position/pose -> odom->base_link tf
add_executable(odom_broadcaster_node src/odom_broadcaster.cpp)
target_link_libraries(odom_broadcaster_node ${catkin_LIBRARIES})

# explore_node: takeoff + frontier exploration + reactive avoidance + altitude hold + save map
# （原算法保留，当前 launch 默认不使用）
add_executable(explore_node src/explore_node.cpp)
target_link_libraries(explore_node ${catkin_LIBRARIES})

# waypoint_mission_node: 直飞航点 + MPPI 局部避障 + tracking PAUSE/RESUME 返回
add_executable(waypoint_mission_node src/waypoint_mission_node.cpp)
target_link_libraries(waypoint_mission_node ${catkin_LIBRARIES})

# area_mission_node: 分区巡逻任务（道路航点 + 区域巡逻 + 深度/激光融合避障）
add_executable(area_mission_node src/area_mission_node.cpp)
target_link_libraries(area_mission_node ${catkin_LIBRARIES})

# scan_self_filter: cut the drone's own body (self-hits) out of the 2D scan
add_executable(scan_self_filter_node src/scan_self_filter.cpp)
target_link_libraries(scan_self_filter_node ${catkin_LIBRARIES})

# scan_map_fuser: fuse N drones' filtered scans into one shared /map (world frame)
add_executable(scan_map_fuser_node src/scan_map_fuser.cpp)
target_link_libraries(scan_map_fuser_node ${catkin_LIBRARIES})

install(TARGETS odom_broadcaster_node explore_node waypoint_mission_node area_mission_node scan_self_filter_node scan_map_fuser_node
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/save_map.sh scripts/road_follow_gpu.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/launch_rviz.py scripts/road_follow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- Tests ----
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(gtest)
  if(gtest_FOUND)
    catkin_add_gtest(mppi_route_selection_test
      test/mppi_route_selection_test.cpp)
    target_link_libraries(mppi_route_selection_test ${catkin_LIBRARIES})
  endif()
endif()
