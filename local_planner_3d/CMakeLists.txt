cmake_minimum_required(VERSION 2.8.3)
project(local_planner_3d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  theta_star
  pcl_conversions
  pcl_ros
  sensor_msgs
  dynamic_reconfigure
)

# Dynamic Reconfigure Parameters
generate_dynamic_reconfigure_options(
  cfg/local_planner.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp theta_star pcl_conversions pcl_ros sensor_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(local_planner_3d src/local_planner_3d.cpp include/local_planner_3d/local_planner_3d.h)
target_link_libraries(local_planner_3d ${catkin_LIBRARIES} ${theta_star_LIBRARIES})
add_dependencies(local_planner_3d ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(dummy_global_planner src/dummy_global_planner.cpp)
target_link_libraries(dummy_global_planner ${catkin_LIBRARIES})
add_dependencies(dummy_global_planner ${catkin_EXPORTED_TARGETS})

add_executable(pointcloud_publisher src/pointcloud_publisher.cpp)
target_link_libraries(pointcloud_publisher ${catkin_LIBRARIES})
add_dependencies(pointcloud_publisher ${catkin_EXPORTED_TARGETS})
