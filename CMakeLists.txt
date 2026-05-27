cmake_minimum_required(VERSION 3.0.2)
project(dynasense)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(dynasense_foot_tof_ray_plugin src/foot_tof_ray_plugin.cpp)
target_link_libraries(dynasense_foot_tof_ray_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dynasense_foot_tof_ray_plugin ${catkin_EXPORTED_TARGETS})

add_library(dynasense_foot_tof_pc_plugin src/foot_tof_ray_pointcloud_plugin.cpp)
target_link_libraries(dynasense_foot_tof_pc_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(dynasense_foot_tof_pc_plugin ${catkin_EXPORTED_TARGETS})

install(TARGETS dynasense_foot_tof_ray_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS dynasense_foot_tof_pc_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
