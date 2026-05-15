cmake_minimum_required(VERSION 3.0.2)
project(my_class_pkg)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
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
  scripts/llm_chat.py
  scripts/start_voice_llm.sh
  scripts/source_voice_llm_env.sh
  scripts/tokenizer.py
  scripts/voice_control.py
  scripts/grab_intent.py
  scripts/test_arm_services.py
  scripts/voice_tag_feedback.py
  scripts/arena_mission.py
  scripts/goto_waypoints_named.py
  scripts/sample_waypoints_enter.py
  scripts/start_mission_two_tasks.sh
  scripts/start_robot_speed_demo.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 实验六脚本（与源码同名 .sh）；安装后：rosrun my_class_pkg exp6_a_bringup.sh
install(PROGRAMS
  scripts/exp6_a_bringup.sh
  scripts/exp6_b_navigation.sh
  scripts/exp6_c_view_nav.sh
  scripts/exp6_d_mission.sh
  scripts/exp6_e_teleop.sh
  scripts/exp6_one_click.sh
  scripts/exp6_run.sh
  scripts/exp6_export_path.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES scripts/exp6_alias.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
