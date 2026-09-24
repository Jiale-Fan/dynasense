cmake_minimum_required(VERSION 3.0.2)
project(dynasense)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_eigen
  tf2_ros
  visualization_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
    BpsState.msg
    GroundClearance.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynasense_bps_map
  CATKIN_DEPENDS
    gazebo_msgs geometry_msgs message_runtime roscpp rospy sensor_msgs std_msgs std_srvs
    tf tf2_eigen tf2_ros visualization_msgs gazebo_ros
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

#############
## Library ##
#############

# Pure geometry: voxel LRU buffers and the BPS query. No ROS, no Gazebo.
add_library(dynasense_bps_map src/bps_map.cpp)
target_link_libraries(dynasense_bps_map ${catkin_LIBRARIES})
add_dependencies(dynasense_bps_map ${catkin_EXPORTED_TARGETS})

###########
## Nodes ##
###########

add_executable(dynasense_bps_node src/bps_node.cpp)
target_link_libraries(dynasense_bps_node dynasense_bps_map ${catkin_LIBRARIES})
add_dependencies(dynasense_bps_node
  dynasense_bps_map
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Depth image -> organized PointCloud2 by pinhole back-projection (no cv_bridge).
add_executable(dynasense_depth_image_to_pointcloud_node src/depth_image_to_pointcloud_node.cpp)
target_link_libraries(dynasense_depth_image_to_pointcloud_node ${catkin_LIBRARIES})
add_dependencies(dynasense_depth_image_to_pointcloud_node ${catkin_EXPORTED_TARGETS})

####################
## Gazebo plugins ##
####################

add_library(dynasense_ground_clearance_plugin src/ground_clearance_plugin.cpp)
target_link_libraries(dynasense_ground_clearance_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dynasense_ground_clearance_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(dynasense_foot_tof_ray_plugin foot_tof/src/foot_tof_ray_plugin.cpp)
target_link_libraries(dynasense_foot_tof_ray_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dynasense_foot_tof_ray_plugin ${catkin_EXPORTED_TARGETS})

add_library(dynasense_foot_tof_pc_plugin foot_tof/src/foot_tof_ray_pointcloud_plugin.cpp)
target_link_libraries(dynasense_foot_tof_pc_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dynasense_foot_tof_pc_plugin ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS
      test/test_frame_alignment.py
      test/test_depth_image_to_pointcloud.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/frame_alignment.test DEPENDENCIES dynasense_bps_node)
  add_rostest(test/depth_image_to_pointcloud.test DEPENDENCIES dynasense_depth_image_to_pointcloud_node)
endif()

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
    foot_tof/bin/tof_preprocessor.py
    scripts/tf_static_aggregator.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Diagnostic shell scripts, runnable with `rosrun dynasense <name>`.
install(PROGRAMS
    scripts/check_contact_sensors.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    dynasense_bps_map
    dynasense_ground_clearance_plugin
    dynasense_foot_tof_ray_plugin
    dynasense_foot_tof_pc_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
    dynasense_bps_node
    dynasense_depth_image_to_pointcloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Installed separately: `foot_tof/launch` would otherwise land on top of
# `launch` (install(DIRECTORY) uses only the last path component).
install(DIRECTORY foot_tof/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/foot_tof/launch
)
