cmake_minimum_required(VERSION 3.0.2)
project(xtdrone_mavros_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/xtdrone_mavros_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/spawn_model_serialized.py
  scripts/wait_for_model.py
  scripts/serial_swarm_bringup.py
  scripts/check_upper_stack.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
