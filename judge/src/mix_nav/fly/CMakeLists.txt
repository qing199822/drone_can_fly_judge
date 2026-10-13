cmake_minimum_required(VERSION 3.0.2)
project(fly)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找catkin和其他依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  topic_tools
)

# 设置包含目录
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fly_takeoff
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs topic_tools
  DEPENDS 
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(fly_takeoff
  src/fly_takeoff.cpp
  src/main.cpp
)

# 链接库
target_link_libraries(fly_takeoff
  ${catkin_LIBRARIES}
)

# 添加依赖
add_dependencies(fly_takeoff
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_fly_launch.py)
endif()

# 安装目标
install(TARGETS fly_takeoff
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
