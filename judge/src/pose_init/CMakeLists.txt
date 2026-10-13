cmake_minimum_required(VERSION 3.0.2)

project(pose_init)

# 查找编译此项目所需的 Catkin 包和组件
# 我们的代码使用了 roscpp, geometry_msgs 和 nav_msgs
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

# 声明 Catkin 包的属性，这对于让其他包依赖于此包很重要
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs
#  DEPENDS system_lib
)


# ${catkin_INCLUDE_DIRS} 是 find_package 找到的依赖包的头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_transformer_node src/pose_init.cpp)

# 指定可执行文件需要链接的库
# ${catkin_LIBRARIES} 包含了所有在 find_package 中列出的组件的库
target_link_libraries(pose_transformer_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/pose_namespace.test)
endif()
