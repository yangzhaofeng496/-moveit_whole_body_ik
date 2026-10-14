cmake_minimum_required(VERSION 3.0.2)
project(moveit_whole_body_ik)

add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  gazebo_msgs
  kdl_parser
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  octomap_msgs
  pcl_conversions
  pluginlib
  roslib
  roscpp
  rviz
  sensor_msgs
  std_srvs
  tf2_ros
  visualization_msgs
  moveit_ros_planning_interface
)

add_service_files(
  FILES
  SolveMoveItIK.srv
  SolveSampledWholeBodyIK.srv
  MoveToPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  gazebo_msgs geometry_msgs kdl_parser message_runtime moveit_core moveit_msgs
    moveit_ros_planning octomap_msgs pcl_conversions pluginlib roslib roscpp
    rviz sensor_msgs std_srvs tf2_ros visualization_msgs
    actionlib control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(CSV_PANEL_MOC
    include/moveit_whole_body_ik/csv_whole_body_ik_panel.hpp)
  set(CSV_PANEL_QT_LIBRARIES ${QT_LIBRARIES})
else()
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  qt5_wrap_cpp(CSV_PANEL_MOC
    include/moveit_whole_body_ik/csv_whole_body_ik_panel.hpp)
  set(CSV_PANEL_QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(moveit_whole_body_ik_csv_panel
  src/csv_whole_body_ik_panel.cpp
  ${CSV_PANEL_MOC}
)
add_dependencies(moveit_whole_body_ik_csv_panel
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_whole_body_ik_csv_panel
  ${catkin_LIBRARIES} ${CSV_PANEL_QT_LIBRARIES})

add_executable(moveit_ik_service_node src/moveit_ik_service_node.cpp)
add_dependencies(moveit_ik_service_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_ik_service_node ${catkin_LIBRARIES})

add_executable(sampled_whole_body_ik_service_node
  src/sampled_whole_body_ik_service_node.cpp)
add_dependencies(sampled_whole_body_ik_service_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sampled_whole_body_ik_service_node ${catkin_LIBRARIES})

add_executable(pcd_octomap_planning_scene_node
  src/pcd_octomap_planning_scene_node.cpp)
add_dependencies(pcd_octomap_planning_scene_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_octomap_planning_scene_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} octomap)

add_executable(moveit_to_gazebo_sync_node src/moveit_to_gazebo_sync_node.cpp)
add_dependencies(moveit_to_gazebo_sync_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_to_gazebo_sync_node ${catkin_LIBRARIES})

add_executable(gazebo_follow_joint_trajectory_node
  src/gazebo_follow_joint_trajectory_node.cpp)
add_dependencies(gazebo_follow_joint_trajectory_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_follow_joint_trajectory_node ${catkin_LIBRARIES})

add_executable(move_group_ready_unpause_gazebo_node
  src/move_group_ready_unpause_gazebo_node.cpp)
add_dependencies(move_group_ready_unpause_gazebo_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_ready_unpause_gazebo_node ${catkin_LIBRARIES})

add_executable(move_to_pose_service_node
  src/move_to_pose_service_node.cpp)
target_link_libraries(move_to_pose_service_node
  ${catkin_LIBRARIES})
add_dependencies(move_to_pose_service_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

install(TARGETS
  moveit_ik_service_node
  sampled_whole_body_ik_service_node
  pcd_octomap_planning_scene_node
  moveit_to_gazebo_sync_node
  gazebo_follow_joint_trajectory_node
  move_group_ready_unpause_gazebo_node
  moveit_whole_body_ik_csv_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES csv_whole_body_ik_panel_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY srv launch data config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
