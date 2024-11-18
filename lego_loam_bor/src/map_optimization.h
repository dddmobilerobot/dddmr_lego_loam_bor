#ifndef MAPOPTIMIZATION_H
#define MAPOPTIMIZATION_H

#include "utility.h"
#include "channel.h"
#include "nanoflann_pcl.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "std_srvs/srv/empty.hpp"

//@allows us to use pcl::transformPointCloud function
#include <pcl/io/pcd_io.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//@ for mkdir
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

//@ for kd tree, used to enhance loop closure robust, we use line of sight test
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//optimized icp gaussian newton
#include "opt_icp_gn/optimized_ICP_GN.h"
#include "opt_icp_gn/common.h"

using namespace std::placeholders;
// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

inline std::string currentDateTime() {
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);
 
    char buffer[128];
    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", now);
    return buffer;
}

class KeyFrames{
  //@ pointcloud
  public:
    void saveKeyFrames(const pcl::PointCloud<PointType>::Ptr normal_feature_cloud, 
                            const pcl::PointCloud<PointType>::Ptr plane_feature_cloud,
                            const tf2::Transform tf2_map2keyframe_pose,
                            const nav_msgs::msg::Odometry feature_odometry);
    std::vector<pcl::PointCloud<PointType>::Ptr> normal_feature_keyframes_;
    std::vector<pcl::PointCloud<PointType>::Ptr> plane_feature_keyframes_;
    std::vector<tf2::Transform> key_poses_;
    std::vector<geometry_msgs::msg::TransformStamped> key_poses_ros_;
    pcl::PointCloud<PointType> key_points_;
    nav_msgs::msg::Odometry feature_odometry_binding_keyframe_;
};

void KeyFrames::saveKeyFrames(const pcl::PointCloud<PointType>::Ptr normal_feature_cloud, 
                            const pcl::PointCloud<PointType>::Ptr plane_feature_cloud,
                            const tf2::Transform tf2_map2keyframe_pose,
                            const nav_msgs::msg::Odometry feature_odometry){

  feature_odometry_binding_keyframe_ = feature_odometry;
  normal_feature_keyframes_.push_back(normal_feature_cloud);
  plane_feature_keyframes_.push_back(plane_feature_cloud);
  key_poses_.push_back(tf2_map2keyframe_pose);

  geometry_msgs::msg::TransformStamped key_pose_ros;
  key_pose_ros.transform.translation.x = tf2_map2keyframe_pose.getOrigin().x();
  key_pose_ros.transform.translation.y = tf2_map2keyframe_pose.getOrigin().y();
  key_pose_ros.transform.translation.z = tf2_map2keyframe_pose.getOrigin().z();
  key_pose_ros.transform.rotation.x = tf2_map2keyframe_pose.getRotation().x();
  key_pose_ros.transform.rotation.y = tf2_map2keyframe_pose.getRotation().y();
  key_pose_ros.transform.rotation.z = tf2_map2keyframe_pose.getRotation().z();
  key_pose_ros.transform.rotation.w = tf2_map2keyframe_pose.getRotation().w();
  key_poses_ros_.push_back(key_pose_ros);

  PointType pt;
  pt.x = tf2_map2keyframe_pose.getOrigin().x();
  pt.y = tf2_map2keyframe_pose.getOrigin().y();
  pt.z = tf2_map2keyframe_pose.getOrigin().z();
  pt.intensity = key_poses_.size()-1;
  key_points_.push_back(pt);
}

class MapOptimization : public rclcpp::Node 
{

 public:
  
  MapOptimization(std::string name, Channel<AssociationOut> &input_channel);
  ~MapOptimization();
  void run();
  void publishGlobalMapThread();

 private:
  
  void initializeMap2KeyframePose();
  void saveKeyFramesAndFactor();
  void updateMap2SensorTF2();
  void extractSurroundingKeyFrames();
  
  //@ everything gtsam
  gtsam::NonlinearFactorGraph isam_graph_;
  gtsam::Values isam_initial_estimate_;
  std::shared_ptr<gtsam::ISAM2> isam_;
  gtsam::Values isam_current_estimation_;
  gtsam::noiseModel::Diagonal::shared_ptr isam_prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr isam_odometry_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr isam_constraint_noise_;
  
  Channel<AssociationOut>& input_channel_;
  
  //@ ros interface: pub/sub/timer
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr timer_run_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_pub_gbl_map_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_run_;
  rclcpp::TimerBase::SharedPtr timer_pub_gbl_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;

  //@ ros msg
  nav_msgs::msg::Odometry feature_odometry_;
  nav_msgs::msg::Odometry feature_odometry_binding_keyframe_;
  
  //@ tf2
  tf2::Transform tf2_map2keyframe_pose_;
  tf2::Transform tf2_map2sensor_;
  tf2::Transform tf2_current_keyframe2sensor_;
  geometry_msgs::msg::TransformStamped tf2_map2sensor_ros_;
  // Key frame
  KeyFrames KF_;
  
  //@ pointcloud
  pcl::PointCloud<PointType>::Ptr current_normal_stitch_at_map_frame_;
  pcl::PointCloud<PointType>::Ptr current_plane_stitch_at_map_frame_;
  pcl::PointCloud<PointType>::Ptr normal_feature_cloud_;
  pcl::PointCloud<PointType>::Ptr plane_feature_cloud_;
  
  //@ voxel
  pcl::VoxelGrid<PointType> ds_pub_map_;
  pcl::VoxelGrid<PointType> ds_current_keyframe_stitch_at_map_frame_;
  pcl::VoxelGrid<PointType> ds_current_feature_frame_;

  LidarSensor lidar_sensor_;
  bool is_first_keyframe_;

};

#endif // MAPOPTIMIZATION_H
