cmake_minimum_required(VERSION 3.5)
project(lego_loam_bor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cloud_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread date_time regex timer chrono)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  pluginlib
  pcl_conversions
  rosbag2_cpp
  rosbag2_storage
  cloud_msgs
  cv_bridge
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

# opt_icp_gn
add_library(opt_icp_gn SHARED
  src/opt_icp_gn/optimized_ICP_GN.cpp
)
target_link_libraries(opt_icp_gn
  ${PCL_LIBRARIES}
)
ament_target_dependencies(opt_icp_gn
  ${dependencies}
)

# mcl/lego loam ip
add_library(image_projection SHARED
  src/image_projection.cpp
)
target_link_libraries(image_projection
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(image_projection
  ${dependencies}
)

#lego loam fa
add_library(feature_association SHARED
  src/feature_association.cpp
)
target_link_libraries(feature_association
  ${PCL_LIBRARIES}
  opt_icp_gn
)
ament_target_dependencies(feature_association
  ${dependencies}
)


#lego loam bag node
add_executable(lego_loam_bag
  src/lego_loam_bag_node.cpp
)
target_link_libraries(lego_loam_bag 
  image_projection
  feature_association
  gtsam
)
ament_target_dependencies(lego_loam_bag
  ${dependencies}
)

install(TARGETS
  opt_icp_gn
  image_projection
  feature_association
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  lego_loam_bag
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  "launch"
  "config"
  "rviz"
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(opt_icp_gn image_projection feature_association map_optimization transform_fusion pose_graph_editor)
ament_export_dependencies(${dependencies})
ament_package()