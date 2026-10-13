cmake_minimum_required(VERSION 3.0.2)
project(look_up)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  RequestTarget.srv
  ReleaseTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 添加Boost库
find_package(Boost REQUIRED COMPONENTS system)

# C++ 可执行文件
add_executable(target_lookup_service 
  src/target_lookup_service.cpp
)

target_link_libraries(target_lookup_service 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(target_lookup_service 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# 安装目标
install(TARGETS target_lookup_service
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)