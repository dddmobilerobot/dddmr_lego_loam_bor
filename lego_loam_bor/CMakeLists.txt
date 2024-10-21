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

find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread date_time regex timer chrono)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED QUIET)

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
  src/imageProjection.cpp
)
target_link_libraries(image_projection
  ${PCL_LIBRARIES}
)
ament_target_dependencies(image_projection
  ${dependencies}
)

#lego loam fa
add_library(feature_association SHARED
  src/featureAssociation.cpp
)
target_link_libraries(feature_association
  ${PCL_LIBRARIES}
)
ament_target_dependencies(feature_association
  ${dependencies}
)

#lego loam mo
add_library(map_optimization SHARED
  src/mapOptimization.cpp
)
target_link_libraries(map_optimization
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  opt_icp_gn
)
ament_target_dependencies(map_optimization
  ${dependencies}
)

#lego loam tf
add_library(transform_fusion SHARED
  src/transformFusion.cpp
)
target_link_libraries(transform_fusion
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
)
ament_target_dependencies(transform_fusion
  ${dependencies}
)

#pose graph editing
add_library(pose_graph_editor SHARED
  src/pose_graph_editor/pose_graph_editor.cpp
)
target_link_libraries(pose_graph_editor
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  opt_icp_gn
)
ament_target_dependencies(pose_graph_editor
  ${dependencies}
)

#pose graph merge editing
add_library(pose_graph_merge_editor SHARED
  src/pose_graph_editor/pose_graph_merge_editor.cpp
)
target_link_libraries(pose_graph_merge_editor
  pose_graph_editor
)
ament_target_dependencies(pose_graph_merge_editor
  ${dependencies}
)

#mcl feature node
add_executable(mcl_feature
  src/mcl_feature_node.cpp
)
target_link_libraries(mcl_feature 
  image_projection
  feature_association
  gtsam
)
ament_target_dependencies(mcl_feature
  ${dependencies}
)

#lego loam bag node
add_executable(lego_loam_bag
  src/lego_loam_bag_node.cpp
)
target_link_libraries(lego_loam_bag 
  image_projection
  feature_association
  map_optimization
  transform_fusion
  gtsam
)
ament_target_dependencies(lego_loam_bag
  ${dependencies}
)

#lego loam node
add_executable(lego_loam
  src/lego_loam_node.cpp
)
target_link_libraries(lego_loam 
  image_projection
  feature_association
  map_optimization
  transform_fusion
  gtsam
)
ament_target_dependencies(lego_loam
  ${dependencies}
)

#pose graph editor node
add_executable(pose_graph_editor_node
  src/pose_graph_editor/pose_graph_editor_node.cpp
)
target_link_libraries(pose_graph_editor_node 
  pose_graph_merge_editor
  gtsam
)
ament_target_dependencies(pose_graph_editor_node
  ${dependencies}
)

install(TARGETS
  opt_icp_gn
  image_projection
  feature_association
  map_optimization
  transform_fusion
  pose_graph_editor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  mcl_feature
  lego_loam
  lego_loam_bag
  pose_graph_editor_node
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