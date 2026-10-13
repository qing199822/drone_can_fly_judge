# ==============================================================================
#                      CMakeLists.txt for the tracking package
# ==============================================================================

cmake_minimum_required(VERSION 3.0.2)
project(tracking)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================= 1. 查找依赖包 =========================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  darknet_ros_msgs
  topic_tools
  look_up
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ========================= 2. Catkin包配置 =========================
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs darknet_ros_msgs topic_tools look_up
  DEPENDS Eigen3
)

# ========================= 3. 构建配置 =========================
# 【修正】: 将 Eigen3::Eigen 添加到 include_directories
# 这一步将 Eigen 库的头文件路径加入到全局包含路径中，
# 确保编译器在任何地方都能找到 #include <Eigen/Dense>。
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} # <--- 修正点 1: 明确添加 Eigen 的 include 路径
)

# ========================= 4. 定义可执行文件 =========================
add_executable(tracking_node
  src/tracking_node.cpp
  src/controller.cpp
  src/state_machine.cpp
  src/kalman_filter.cpp
  src/kalman_filter_base.cpp
  src/service_manager.cpp
  src/smoothing.cpp
)

add_dependencies(tracking_node ${catkin_EXPORTED_TARGETS})

# ========================= 5. 链接库 =========================
# 【修正】: 将 Eigen3::Eigen 添加到 target_link_libraries
# 这是更现代、更推荐的做法。它不仅处理链接（如果需要），
# 还会自动将 Eigen 的 include 目录和编译选项应用到 tracking_node 这个目标上。
target_link_libraries(tracking_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen # <--- 修正点 2: 链接 Eigen 目标
)

# ========================= 6. 安装规则 (可选但推荐) =========================
install(TARGETS tracking_node
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
