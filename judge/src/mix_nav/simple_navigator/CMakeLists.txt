# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.0.2)
# 定义项目名称，应与 package.xml 中的 <name> 标签一致
project(simple_navigator)

# 查找编译此项目所需的Catkin依赖项
# 这些组件必须与 package.xml 中声明的依赖项相对应
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
)

#===========================================================================
# Catkin 包配置
#===========================================================================
# 这个宏是catkin包所必需的。
# CATKIN_DEPENDS: 声明了此包的ROS依赖项，以便其他包可以找到它们。
# INCLUDE_DIRS: 声明了此包导出的头文件目录，以便其他包可以使用它们。
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs tf
)

#===========================================================================
# 包含目录
#===========================================================================
# 添加头文件目录，以便编译器可以找到 #include <...>
# 'include': 用于我们自己的头文件 (simple_navigator.h)
# '${catkin_INCLUDE_DIRS}': 用于来自catkin依赖项的头文件 (e.g., ros/ros.h)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#===========================================================================
# 构建可执行文件
#===========================================================================
# 定义一个可执行文件
# 参数1: 可执行文件的名称 (这个名称将用于 rosrun 和 launch 文件中的 'type')
# 参数2...: 用于构建此可执行文件的源文件列表
add_executable(simple_navigator_node src/simple_navigator.cpp)

# 将可执行文件链接到它需要的库
# ${catkin_LIBRARIES} 包含了所有在 find_package 中找到的依赖项的库
target_link_libraries(simple_navigator_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/velocity_continuity.test)
endif()

#===========================================================================
# 安装规则 (可选，但推荐)
#===========================================================================
# 这些规则定义了当运行 'catkin_make install' 时，哪些文件应该被安装
install(TARGETS simple_navigator_node
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
