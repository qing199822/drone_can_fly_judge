#==============================================================================
#==                       CMake Version and Project Name                     ==
#==============================================================================
cmake_minimum_required(VERSION 3.0.2)
project(transform_tree)

#==============================================================================
#==                       Find Dependencies (Catkin)                         ==
#==============================================================================
# 查找编译本包所需的其他ROS包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf2_msgs
)

#==============================================================================
#==                  Catkin Package Information (for others)                 ==
#==============================================================================
# 声明包的属性
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf2_msgs
)

#==============================================================================
#==                            Build Configuration                           ==
#==============================================================================
# 指定头文件搜索路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 声明要生成的可执行文件
add_executable(tf_broadcaster_node
  src/tf_broadcaster_node.cpp
  src/manual_tf_broadcaster.cpp
)

# 链接可执行文件到它所需要的库
target_link_libraries(tf_broadcaster_node
  ${catkin_LIBRARIES}
)

#==============================================================================
#==                               Installation                               ==
#==============================================================================
# 安装可执行文件
install(TARGETS tf_broadcaster_node
  DESTINATION ${CATKIN_PACKAGE_LIB_DIR}
)

# 安装launch文件夹
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DIR}/
)

# === FIX STARTS HERE ===
# 安装头文件 (修正后的、更健壮的版本)
# 我们明确指定要安装哪一个文件，以及它要被安装到哪个目标子目录下。
install(FILES include/transform_tree/manual_tf_broadcaster.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DIR}/transform_tree
)
# === FIX ENDS HERE ===
