cmake_minimum_required(VERSION 3.0.2)
project(my_class_pkg)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  tf2_ros
  tf2_geometry_msgs
  upros_message
)

generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Logging node (C++)
add_executable(ros_log src/ros_log.cpp)
target_link_libraries(ros_log ${catkin_LIBRARIES})

add_executable(ros_dynamic_speed_node src/ros_dynamic_speed.cpp)
add_dependencies(ros_dynamic_speed_node ${PROJECT_NAME}_gencfg)
target_link_libraries(ros_dynamic_speed_node ${catkin_LIBRARIES})

add_executable(tag_grab_node src/tag_grab.cpp)
add_dependencies(tag_grab_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tag_grab_node ${catkin_LIBRARIES})

# Install targets
install(TARGETS ros_log ros_dynamic_speed_node tag_grab_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/ros_log.py
  scripts/follow_line.py
  scripts/get_ros_image.py
  scripts/upros_gesture.py
  scripts/gesture_movement.py
  scripts/apriltag_follow.py
  scripts/start_robot_speed_demo.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
