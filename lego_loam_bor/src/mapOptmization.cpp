// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "mapOptimization.h"
#include <future>

using namespace gtsam;

MapOptimization::MapOptimization(std::string name,
                                 Channel<AssociationOut> &input_channel)
    : Node(name),
      _input_channel(input_channel),
      odom_type_("")
{
  
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);
  pose_graph_.clear();

  srvSavePCD = this->create_service<std_srvs::srv::Empty>("save_mapped_point_cloud", std::bind(&MapOptimization::pcdSaver, this, std::placeholders::_1, std::placeholders::_2));

  pub_key_pose_arr_ = this->create_publisher<geometry_msgs::msg::PoseArray>("key_poses", 1);
  
  pub_pose_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_surround", 1);  
  
  pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 5);  
  
  pubHistoryKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("history_cloud", 1);  

  pubIcpKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_cloud", 1);  

  pubRecentKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("recent_cloud", 1);  

  //TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  clock_ = this->get_clock();

  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
  downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

  // for histor key frames of loop closure
  downSizeFilterHistoryKeyFrames.setLeafSize(0.4, 0.4, 0.4);
  // for surrounding key poses of scan-to-map optimization
  downSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0);

  // for global map visualization
  downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0);
  // for global map visualization
  downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4);

  // DS for final stitch
  downSizeFilterFinalStitch.setLeafSize(0.1, 0.1, 0.1);

  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";

  aftMappedTrans.header.frame_id = "camera_init";
  aftMappedTrans.child_frame_id = "aft_mapped";

  declare_parameter("mapping.enable_loop_closure", rclcpp::ParameterValue(false));
  this->get_parameter("mapping.enable_loop_closure", _loop_closure_enabled);
  RCLCPP_INFO(this->get_logger(), "mapping.enable_loop_closure: %d", _loop_closure_enabled);

  declare_parameter("mapping.history_keyframe_search_radius", rclcpp::ParameterValue(0.0));
  this->get_parameter("mapping.history_keyframe_search_radius", _history_keyframe_search_radius);
  RCLCPP_INFO(this->get_logger(), "mapping.history_keyframe_search_radius: %.2f", _history_keyframe_search_radius);

  declare_parameter("mapping.history_keyframe_search_num", rclcpp::ParameterValue(0));
  this->get_parameter("mapping.history_keyframe_search_num", _history_keyframe_search_num);
  RCLCPP_INFO(this->get_logger(), "mapping.history_keyframe_search_num: %d", _history_keyframe_search_num);

  declare_parameter("mapping.history_keyframe_fitness_score", rclcpp::ParameterValue(0.0));
  this->get_parameter("mapping.history_keyframe_fitness_score", _history_keyframe_fitness_score);
  RCLCPP_INFO(this->get_logger(), "mapping.history_keyframe_fitness_score: %.2f", _history_keyframe_fitness_score);

  declare_parameter("mapping.surrounding_keyframe_search_radius", rclcpp::ParameterValue(0.0));
  this->get_parameter("mapping.surrounding_keyframe_search_radius", _surrounding_keyframe_search_radius);
  RCLCPP_INFO(this->get_logger(), "mapping.surrounding_keyframe_search_radius: %.2f", _surrounding_keyframe_search_radius);

  declare_parameter("mapping.surrounding_keyframe_search_num", rclcpp::ParameterValue(0));
  this->get_parameter("mapping.surrounding_keyframe_search_num", _surrounding_keyframe_search_num);
  RCLCPP_INFO(this->get_logger(), "mapping.surrounding_keyframe_search_num: %d", _surrounding_keyframe_search_num);

  declare_parameter("mapping.global_map_visualization_search_radius", rclcpp::ParameterValue(0.0));
  this->get_parameter("mapping.global_map_visualization_search_radius", _global_map_visualization_search_radius);
  RCLCPP_INFO(this->get_logger(), "mapping.global_map_visualization_search_radius: %.2f", _global_map_visualization_search_radius);

  declare_parameter("mapping.angle_between_key_frame", rclcpp::ParameterValue(0.3));
  this->get_parameter("mapping.angle_between_key_frame", angle_between_key_frame_);
  RCLCPP_INFO(this->get_logger(), "mapping.angle_between_key_frame: %.2f", angle_between_key_frame_);

  declare_parameter("mapping.distance_between_key_frame", rclcpp::ParameterValue(0.3));
  this->get_parameter("mapping.distance_between_key_frame", distance_between_key_frame_);
  RCLCPP_INFO(this->get_logger(), "mapping.distance_between_key_frame: %.2f", distance_between_key_frame_);

  
  declare_parameter("featureAssociation.odom_type", rclcpp::ParameterValue(""));
  this->get_parameter("featureAssociation.odom_type", odom_type_);
  RCLCPP_INFO(this->get_logger(), "featureAssociation.odom_type: %s", odom_type_.c_str());

  allocateMemory();

  timer_run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_pub_gbl_map_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_loop_closure_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_run_ = this->create_wall_timer(1ms, std::bind(&MapOptimization::run, this), timer_run_cb_group_);
  timer_pub_gbl_map_ = this->create_wall_timer(200ms, std::bind(&MapOptimization::publishGlobalMapThread, this), timer_pub_gbl_map_cb_group_);
  timer_loop_closure_ = this->create_wall_timer(1000ms, std::bind(&MapOptimization::loopClosureThread, this), timer_loop_closure_cb_group_);
  
  //@ Generate map2camera_init. This will be used to export final point cloud/pose graph based on map frame
  //@ "0 0 0 pi/2 0 pi/2 map -> camera_init" and "0 0 0 -pi/2 -pi/2 0 camera -> base_link"
  //@affine_3d for pcl conversion, tf2::stamped for key_pose
  geometry_msgs::msg::TransformStamped trans_m2ci;
  trans_m2ci.transform.translation.x = 0.0; trans_m2ci.transform.translation.y = 0.0; trans_m2ci.transform.translation.z = 0.0;
  trans_m2ci.transform.rotation.x = 0.5; trans_m2ci.transform.rotation.y = 0.5;
  trans_m2ci.transform.rotation.z = 0.5; trans_m2ci.transform.rotation.w = 0.5;

  geometry_msgs::msg::TransformStamped trans_c2baselink;
  trans_c2baselink.transform.translation.x = 0.0; trans_c2baselink.transform.translation.y = 0.0; trans_c2baselink.transform.translation.z = 0.0;
  trans_c2baselink.transform.rotation.x = -0.5; trans_c2baselink.transform.rotation.y = -0.5;
  trans_c2baselink.transform.rotation.z = -0.5; trans_c2baselink.transform.rotation.w = 0.5;

  trans_m2ci_af3_ = tf2::transformToEigen(trans_m2ci); //for pcl conversion
  trans_c2b_af3_ = tf2::transformToEigen(trans_c2baselink); //for pcl conversion
  trans_b2c_af3_ = trans_c2b_af3_.inverse();
  //for key_pose
  tf2_trans_m2ci_.setRotation(tf2::Quaternion(trans_m2ci.transform.rotation.x, trans_m2ci.transform.rotation.y, trans_m2ci.transform.rotation.z, trans_m2ci.transform.rotation.w));
  tf2_trans_m2ci_.setOrigin(tf2::Vector3(trans_m2ci.transform.translation.x, trans_m2ci.transform.translation.y, trans_m2ci.transform.translation.z));

  tf2_trans_c2b_.setRotation(tf2::Quaternion(trans_c2baselink.transform.rotation.x, trans_c2baselink.transform.rotation.y, trans_c2baselink.transform.rotation.z, trans_c2baselink.transform.rotation.w));
  tf2_trans_c2b_.setOrigin(tf2::Vector3(trans_c2baselink.transform.translation.x, trans_c2baselink.transform.translation.y, trans_c2baselink.transform.translation.z));

}

void MapOptimization::pcdSaver(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response){
  
  if (cloudKeyPoses3D->points.empty() == true) 
    return;

  std::string mapping_dir_string;
  auto env_p = std::getenv("DDDMR_MAPPING_DIR");
  if ( env_p == NULL ) {
    mapping_dir_string = std::string("/tmp/") + currentDateTime();
    std::filesystem::create_directory(mapping_dir_string);
    RCLCPP_INFO(this->get_logger(), "Create dir: %s", mapping_dir_string.c_str());
  } else {
    mapping_dir_string = std::string( env_p ) + currentDateTime();
    std::filesystem::create_directory(mapping_dir_string);
    RCLCPP_INFO(this->get_logger(), "Create dir: %s", mapping_dir_string.c_str());
  }


  // save map
  for (int i = 0; i < cloudKeyPoses3D->points.size(); ++i) {
    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
    *completeGlobalStitch += *transformPointCloud(
        cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*completeGlobalStitch += *transformPointCloud(
    //    surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*completeGlobalStitch +=
    //    *transformPointCloud(outlierCloudKeyFrames[thisKeyInd],
    //                         &cloudKeyPoses6D->points[thisKeyInd]);
  }

  downSizeFilterFinalStitch.setInputCloud(completeGlobalStitch);
  downSizeFilterFinalStitch.filter(*completeGlobalStitch);
  pcl::io::savePCDFileASCII(mapping_dir_string + "/map.pcd", *completeGlobalStitch);
  
  //save surface
  completeGlobalStitch.reset(new pcl::PointCloud<PointType>());
  for (int i = 0; i < cloudKeyPoses3D->points.size(); ++i) {
    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
    *completeGlobalStitch += *transformPointCloud(
        surfFlatCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
  }
  downSizeFilterFinalStitch.setInputCloud(completeGlobalStitch);
  downSizeFilterFinalStitch.filter(*completeGlobalStitch);
  pcl::io::savePCDFileASCII(mapping_dir_string + "/ground.pcd", *completeGlobalStitch);
  
  completeGlobalStitch.reset(new pcl::PointCloud<PointType>());
  for (int i = 0; i < cloudKeyPoses3D->points.size(); ++i) {
    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
    *completeGlobalStitch += *transformPointCloud(
        cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    *completeGlobalStitch += *transformPointCloud(
        surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*completeGlobalStitch +=
    //    *transformPointCloud(outlierCloudKeyFrames[thisKeyInd],
    //                         &cloudKeyPoses6D->points[thisKeyInd]);
  }
  
  //@ -----Write poses-----
  pcl::PointCloud<PointTypePose> cloudKeyPoses6DBaseLink;
  for(auto it = cloudKeyPoses6D->points.begin(); it!=cloudKeyPoses6D->points.end(); it++){
    tf2::Transform key_pose_ci2c;
    tf2::Quaternion q;
    q.setRPY( (*it).roll, (*it).pitch, (*it).yaw);
    key_pose_ci2c.setRotation(q);
    key_pose_ci2c.setOrigin(tf2::Vector3((*it).x, (*it).y, (*it).z));

    tf2::Transform key_pose_ci2b;
    key_pose_ci2b.mult(key_pose_ci2c, tf2_trans_c2b_);

    tf2::Transform key_pose_m2b;
    key_pose_m2b.mult(tf2_trans_m2ci_, key_pose_ci2b);
    
    PointTypePose pt;
    pt.x = key_pose_m2b.getOrigin().x();
    pt.y = key_pose_m2b.getOrigin().y();
    pt.z = key_pose_m2b.getOrigin().z();
    tf2::Matrix3x3 m(key_pose_m2b.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pt.roll = roll;
    pt.pitch = pitch;
    pt.yaw = yaw;
    pt.intensity = (*it).intensity;
    cloudKeyPoses6DBaseLink.push_back(pt);
  }  
  pcl::io::savePCDFileASCII(mapping_dir_string + "/poses.pcd", cloudKeyPoses6DBaseLink);
  
  //@ -----Write graph-----
  pcl::PointCloud<pcl::PointXYZ> edges;
  int edge_number = 0;
  for(auto it = pose_graph_.begin(); it!=pose_graph_.end(); it++){
    pcl::PointXYZ pt;
    pt.x = edge_number;
    pt.y = (*it).first;
    pt.z = (*it).second;
    edges.push_back(pt);
    edge_number++;
  }
  pcl::io::savePCDFileASCII(mapping_dir_string + "/edges.pcd", edges);

  //@ -----Write pcd-----
  std::string pcd_dir = mapping_dir_string + "/pcd";
  std::filesystem::create_directory(pcd_dir);
  for (int i = 0; i < cloudKeyPoses6D->points.size(); ++i) {
    keyFrameStitch.reset(new pcl::PointCloud<PointType>());
    int thisKeyInd = (int)cloudKeyPoses6D->points[i].intensity;
    *keyFrameStitch += (*cornerCloudKeyFrames[thisKeyInd]);
    pcl::transformPointCloud(*keyFrameStitch, *keyFrameStitch, trans_b2c_af3_);
    pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_feature.pcd", *keyFrameStitch);

    keyFrameGroundStitch.reset(new pcl::PointCloud<PointType>());
    *keyFrameGroundStitch += (*surfFlatCloudKeyFrames[thisKeyInd]);
    pcl::transformPointCloud(*keyFrameGroundStitch, *keyFrameGroundStitch, trans_b2c_af3_);
    pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_ground.pcd", *keyFrameGroundStitch);

  }  


}


MapOptimization::~MapOptimization()
{
  _input_channel.send({});
}


void MapOptimization::allocateMemory() {
  cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

  surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
  surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

  laserCloudCornerLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudSurfLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudSurfFlatLast.reset(
      new pcl::PointCloud<PointType>());  // for ground pcd stitching
                                          // odoOptimization
  laserCloudCornerLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner featuer set
                                          // from odoOptimization
  laserCloudSurfLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization
  laserCloudOutlierLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudOutlierLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner feature set
                                          // from odoOptimization
  laserCloudSurfTotalLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudSurfTotalLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization

  laserCloudOri.reset(new pcl::PointCloud<PointType>());
  coeffSel.reset(new pcl::PointCloud<PointType>());

  laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

  nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
  nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

  latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

  globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
  globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
  globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
  globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());
  completeGlobalStitch.reset(new pcl::PointCloud<PointType>());
  keyFrameStitch.reset(new pcl::PointCloud<PointType>());

  timeLaserOdometry = 0;
  timeLastGloalMapPublish = 0;

  for (int i = 0; i < 6; ++i) {
    transformLast[i] = 0;
    transformSum[i] = 0;
    transformIncre[i] = 0;
    transformTobeMapped[i] = 0;
    transformBefMapped[i] = 0;
    transformAftMapped[i] = 0;
  }

  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  priorNoise = noiseModel::Diagonal::Variances(Vector6);
  odometryNoise = noiseModel::Diagonal::Variances(Vector6);

  matA0.setZero();
  matB0.fill(-1);
  matX0.setZero();

  matA1.setZero();
  matD1.setZero();
  matV1.setZero();

  isDegenerate = false;
  matP.setZero();

  laserCloudCornerFromMapDSNum = 0;
  laserCloudSurfFromMapDSNum = 0;
  laserCloudCornerLastDSNum = 0;
  laserCloudSurfLastDSNum = 0;
  laserCloudOutlierLastDSNum = 0;
  laserCloudSurfTotalLastDSNum = 0;

  potentialLoopFlag = false;
  aLoopIsClosed = false;

  latestFrameID = 0;
}


void MapOptimization::publishGlobalMapThread()
{
  publishGlobalMap();
}


void MapOptimization::loopClosureThread()
{
  if(_loop_closure_enabled){
    performLoopClosure();
  }
}


void MapOptimization::transformAssociateToMap() {
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) -
             sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) +
             cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz +
                       calx * calz * cblx * cblz) -
              cbcx * sbcy *
                  (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                   calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                   cblx * salx * sbly) -
              cbcx * cbcy *
                  (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                   calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                   cblx * cbly * salx);
  transformTobeMapped[0] = -asin(srx);

  float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) -
                         cblx * sblz * (caly * calz + salx * saly * salz) +
                         calx * saly * sblx) -
                 cbcx * cbcy *
                     ((caly * calz + salx * saly * salz) *
                          (cblz * sbly - cbly * sblx * sblz) +
                      (caly * salz - calz * salx * saly) *
                          (sbly * sblz + cbly * cblz * sblx) -
                      calx * cblx * cbly * saly) +
                 cbcx * sbcy *
                     ((caly * calz + salx * saly * salz) *
                          (cbly * cblz + sblx * sbly * sblz) +
                      (caly * salz - calz * salx * saly) *
                          (cbly * sblz - cblz * sblx * sbly) +
                      calx * cblx * saly * sbly);
  float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) -
                         cblx * cblz * (saly * salz + caly * calz * salx) +
                         calx * caly * sblx) +
                 cbcx * cbcy *
                     ((saly * salz + caly * calz * salx) *
                          (sbly * sblz + cbly * cblz * sblx) +
                      (calz * saly - caly * salx * salz) *
                          (cblz * sbly - cbly * sblx * sblz) +
                      calx * caly * cblx * cbly) -
                 cbcx * sbcy *
                     ((saly * salz + caly * calz * salx) *
                          (cbly * sblz - cblz * sblx * sbly) +
                      (calz * saly - caly * salx * salz) *
                          (cbly * cblz + sblx * sbly * sblz) -
                      calx * caly * cblx * sbly);
  transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                 crycrx / cos(transformTobeMapped[0]));

  float srzcrx =
      (cbcz * sbcy - cbcy * sbcx * sbcz) *
          (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
           calx * calz * (sbly * sblz + cbly * cblz * sblx) +
           cblx * cbly * salx) -
      (cbcy * cbcz + sbcx * sbcy * sbcz) *
          (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
           calx * salz * (cbly * cblz + sblx * sbly * sblz) +
           cblx * salx * sbly) +
      cbcx * sbcz *
          (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  float crzcrx =
      (cbcy * sbcz - cbcz * sbcx * sbcy) *
          (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
           calx * salz * (cbly * cblz + sblx * sbly * sblz) +
           cblx * salx * sbly) -
      (sbcy * sbcz + cbcy * cbcz * sbcx) *
          (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
           calx * calz * (sbly * sblz + cbly * cblz * sblx) +
           cblx * cbly * salx) +
      cbcx * cbcz *
          (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                 crzcrx / cos(transformTobeMapped[0]));

  x1 = cos(transformTobeMapped[2]) * transformIncre[3] -
       sin(transformTobeMapped[2]) * transformIncre[4];
  y1 = sin(transformTobeMapped[2]) * transformIncre[3] +
       cos(transformTobeMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  transformTobeMapped[3] =
      transformAftMapped[3] -
      (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
  transformTobeMapped[4] = transformAftMapped[4] - y2;
  transformTobeMapped[5] =
      transformAftMapped[5] -
      (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void MapOptimization::transformUpdate() {
 
  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = transformSum[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
  //Force pitch and roll to robot pose
  if(odom_type_ == "wheel_odometry"){
    transformAftMapped[0] = transformSum[0];
    transformAftMapped[2] = transformSum[2];
  }
}

void MapOptimization::updatePointAssociateToMapSinCos() {
  cRoll = cos(transformTobeMapped[0]);
  sRoll = sin(transformTobeMapped[0]);

  cPitch = cos(transformTobeMapped[1]);
  sPitch = sin(transformTobeMapped[1]);

  cYaw = cos(transformTobeMapped[2]);
  sYaw = sin(transformTobeMapped[2]);

  tX = transformTobeMapped[3];
  tY = transformTobeMapped[4];
  tZ = transformTobeMapped[5];
}

void MapOptimization::pointAssociateToMap(PointType const *const pi,
                                          PointType *const po) {
  float x1 = cYaw * pi->x - sYaw * pi->y;
  float y1 = sYaw * pi->x + cYaw * pi->y;
  float z1 = pi->z;

  float x2 = x1;
  float y2 = cRoll * y1 - sRoll * z1;
  float z2 = sRoll * y1 + cRoll * z1;

  po->x = cPitch * x2 + sPitch * z2 + tX;
  po->y = y2 + tY;
  po->z = -sPitch * x2 + cPitch * z2 + tZ;
  po->intensity = pi->intensity;
}

void MapOptimization::updateTransformPointCloudSinCos(PointTypePose *tIn) {
  ctRoll = cos(tIn->roll);
  stRoll = sin(tIn->roll);

  ctPitch = cos(tIn->pitch);
  stPitch = sin(tIn->pitch);

  ctYaw = cos(tIn->yaw);
  stYaw = sin(tIn->yaw);

  tInX = tIn->x;
  tInY = tIn->y;
  tInZ = tIn->z;
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn) {
  // !!! DO NOT use pcl for point cloud transformation, results are not
  // accurate Reason: unkown
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType *pointFrom;
  PointType pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
    float y1 = stYaw * pointFrom->x + ctYaw * pointFrom->y;
    float z1 = pointFrom->z;

    float x2 = x1;
    float y2 = ctRoll * y1 - stRoll * z1;
    float z2 = stRoll * y1 + ctRoll * z1;

    pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
    pointTo.y = y2 + tInY;
    pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
    pointTo.intensity = pointFrom->intensity;

    cloudOut->points[i] = pointTo;
  }
  return cloudOut;
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType *pointFrom;
  PointType pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    float x1 = cos(transformIn->yaw) * pointFrom->x -
               sin(transformIn->yaw) * pointFrom->y;
    float y1 = sin(transformIn->yaw) * pointFrom->x +
               cos(transformIn->yaw) * pointFrom->y;
    float z1 = pointFrom->z;

    float x2 = x1;
    float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
    float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

    pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 +
                transformIn->x;
    pointTo.y = y2 + transformIn->y;
    pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 +
                transformIn->z;
    pointTo.intensity = pointFrom->intensity;

    cloudOut->points[i] = pointTo;
  }
  return cloudOut;
}


void MapOptimization::publishTF() {

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);
  geometry_msgs::msg::Quaternion geoQuat;
  tf2::convert(quat_tf, geoQuat);

  odomAftMapped.header.stamp = timeLaserOdometry_header_.stamp;
  odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
  odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
  odomAftMapped.pose.pose.orientation.z = geoQuat.x;
  odomAftMapped.pose.pose.orientation.w = geoQuat.w;
  odomAftMapped.pose.pose.position.x = transformAftMapped[3];
  odomAftMapped.pose.pose.position.y = transformAftMapped[4];
  odomAftMapped.pose.pose.position.z = transformAftMapped[5];
  odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
  odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
  odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
  odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
  odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
  odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
  pubOdomAftMapped->publish(odomAftMapped);

  aftMappedTrans.header.stamp = timeLaserOdometry_header_.stamp;
  aftMappedTrans.transform.rotation.x = -geoQuat.y;
  aftMappedTrans.transform.rotation.y = -geoQuat.z;
  aftMappedTrans.transform.rotation.z = geoQuat.x;
  aftMappedTrans.transform.rotation.w = geoQuat.w;
  aftMappedTrans.transform.translation.x = transformAftMapped[3];
  aftMappedTrans.transform.translation.y = transformAftMapped[4];
  aftMappedTrans.transform.translation.z = transformAftMapped[5];

  tf_broadcaster_->sendTransform(aftMappedTrans);
}

void MapOptimization::publishKeyPosesAndFrames() {

  if (true) {
    geometry_msgs::msg::PoseArray pose_array;
    for(auto it = cloudKeyPoses6D->points.begin(); it!=cloudKeyPoses6D->points.end(); it++){
      tf2::Transform key_pose_ci2c;
      tf2::Quaternion q;
      q.setRPY( (*it).roll, (*it).pitch, (*it).yaw);
      key_pose_ci2c.setRotation(q);
      key_pose_ci2c.setOrigin(tf2::Vector3((*it).x, (*it).y, (*it).z));

      tf2::Transform key_pose_ci2b;
      key_pose_ci2b.mult(key_pose_ci2c, tf2_trans_c2b_);

      tf2::Transform key_pose_m2b;
      key_pose_m2b.mult(tf2_trans_m2ci_, key_pose_ci2b);

      geometry_msgs::msg::Pose a_pose;
      a_pose.position.x = key_pose_m2b.getOrigin().x();
      a_pose.position.y = key_pose_m2b.getOrigin().y();
      a_pose.position.z = key_pose_m2b.getOrigin().z();
      a_pose.orientation.x = key_pose_m2b.getRotation().x();
      a_pose.orientation.y = key_pose_m2b.getRotation().y();
      a_pose.orientation.z = key_pose_m2b.getRotation().z();
      a_pose.orientation.w = key_pose_m2b.getRotation().w();
      pose_array.poses.push_back(a_pose);
    }
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = clock_->now();
    pub_key_pose_arr_->publish(pose_array);
    
    //@visualize edge
    visualization_msgs::msg::MarkerArray markerArray;
    int cnt = 0;
    for(auto it=pose_graph_.begin();it!=pose_graph_.end();it++){
      visualization_msgs::msg::Marker markerEdge;
      markerEdge.header.frame_id = "map";
      markerEdge.header.stamp = clock_->now();
      markerEdge.action = visualization_msgs::msg::Marker::MODIFY;
      markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
      markerEdge.pose.orientation.w = 1.0;
      markerEdge.ns = "edges";
      markerEdge.id = 0;
      markerEdge.scale.x = 0.25;markerEdge.scale.y = 0.25;markerEdge.scale.z = 0.25;
      markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
      markerEdge.color.a = 0.9;
      geometry_msgs::msg::Point p1;
      geometry_msgs::msg::Point p2;
      p1.x = pose_array.poses[(*it).first].position.x;
      p1.y = pose_array.poses[(*it).first].position.y;
      p1.z = pose_array.poses[(*it).first].position.z; //make edge under node, make visualization easier
      p2.x = pose_array.poses[(*it).second].position.x;
      p2.y = pose_array.poses[(*it).second].position.y;
      p2.z = pose_array.poses[(*it).second].position.z;//make edge under node, make visualization easier
      markerEdge.points.push_back(p1);
      markerEdge.points.push_back(p2);
      markerEdge.id = cnt;
      markerArray.markers.push_back(markerEdge);
      cnt++;
    }

    cnt = 0;
    for(auto it=pose_array.poses.begin(); it!=pose_array.poses.end(); it++){

      visualization_msgs::msg::Marker markerPoint;
      markerPoint.header.frame_id = "map";
      markerPoint.header.stamp = clock_->now();
      markerPoint.action = visualization_msgs::msg::Marker::MODIFY;
      markerPoint.type = visualization_msgs::msg::Marker::SPHERE;
      markerPoint.ns = "nodes";
      markerPoint.id = 0;
      markerPoint.scale.x = 0.25; markerPoint.scale.y = 0.25; markerPoint.scale.z = 0.25;
      markerPoint.color.r = 0.1; markerPoint.color.g = 0.1; markerPoint.color.b = 1;
      markerPoint.color.a = 0.8; 

      markerPoint.pose = (*it);
      markerPoint.id = cnt;
      markerArray.markers.push_back(markerPoint);
      cnt++;
    }
    pub_pose_graph_->publish(markerArray);
  }

  if (false) {
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry_header_.stamp;
    cloudMsgTemp.header.frame_id = "camera_init";
    pubRecentKeyFrames->publish(cloudMsgTemp);
  }
}

void MapOptimization::publishGlobalMap() {

  if (cloudKeyPoses3D->points.empty() == true) return;
  // kd-tree to find near key frames to visualize
  std::vector<int> pointSearchIndGlobalMap;
  std::vector<float> pointSearchSqDisGlobalMap;
  // search near key frames to visualize
  mtx.lock();
  kdtreeGlobalMap.setInputCloud(cloudKeyPoses3D);
  kdtreeGlobalMap.radiusSearch(
      currentRobotPosPoint, _global_map_visualization_search_radius,
      pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
  mtx.unlock();

  for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
    globalMapKeyPoses->points.push_back(
        cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
  // downsample near selected key frames
  downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
  downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
  // extract visualized and downsampled key frames
  for (int i = 0; i < globalMapKeyPosesDS->points.size(); ++i) {
    int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
    *globalMapKeyFrames += *transformPointCloud(
        cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    *globalMapKeyFrames += *transformPointCloud(
        surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*globalMapKeyFrames +=
    //    *transformPointCloud(outlierCloudKeyFrames[thisKeyInd],
    //                         &cloudKeyPoses6D->points[thisKeyInd]);
  }
  // downsample visualized points
  downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
  downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
  
  //@ transform to map frame --> z pointing to sky
  
  pcl::transformPointCloud(*globalMapKeyFramesDS, *globalMapKeyFramesDS, trans_m2ci_af3_);
  sensor_msgs::msg::PointCloud2 cloudMsgTemp;
  pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
  cloudMsgTemp.header.stamp = timeLaserOdometry_header_.stamp;
  cloudMsgTemp.header.frame_id = "map";
  pubLaserCloudSurround->publish(cloudMsgTemp);

  globalMapKeyPoses->clear();
  globalMapKeyPosesDS->clear();
  globalMapKeyFrames->clear();
  //globalMapKeyFramesDS->clear();
}

bool MapOptimization::detectLoopClosure() {
  latestSurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloudDS->clear();

  std::lock_guard<std::mutex> lock(mtx);
  // find the closest history key frame
  std::vector<int> pointSearchIndLoop;
  std::vector<float> pointSearchSqDisLoop;
  kdtreeHistoryKeyPoses.setInputCloud(cloudKeyPoses3D);
  kdtreeHistoryKeyPoses.radiusSearch(
      currentRobotPosPoint, _history_keyframe_search_radius, pointSearchIndLoop,
      pointSearchSqDisLoop);

  closestHistoryFrameID = -1;
  for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
    int id = pointSearchIndLoop[i];

    double dx = cloudKeyPoses6D->points[id].x - transformLast[3];
    double dy = cloudKeyPoses6D->points[id].y - transformLast[4];
    double dz = cloudKeyPoses6D->points[id].z - transformLast[5];

    double droll = cloudKeyPoses6D->points[id].roll - transformLast[0];
    double dpitch = cloudKeyPoses6D->points[id].pitch - transformLast[1];
    double dyaw = cloudKeyPoses6D->points[id].yaw - transformLast[2];
    
    double d_distance = sqrt(dx*dx + dy *dy + dz*dz);
    double d_theta = sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw);

    if (d_distance<15.0 && cloudKeyPoses6D->points.size() - id > 20){
      closestHistoryFrameID = id;
      RCLCPP_INFO(this->get_logger(), "Closest key id: %d, %.2f, %.2f", closestHistoryFrameID, d_distance, d_theta);
      break;
    }

  }
  if (closestHistoryFrameID == -1) {
    return false;
  }

  // save latest key frames
  latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
  *latestSurfKeyFrameCloud +=
      *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure],
                           &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  *latestSurfKeyFrameCloud +=
      *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],
                           &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  
  //@ without transform function in lego loam, the original frame is camera in cornerCloudKeyFrames/surfCloudKeyFrames
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudBaseLinkFrame(new pcl::PointCloud<PointType>());

  *latestSurfKeyFrameCloudBaseLinkFrame += (*cornerCloudKeyFrames[latestFrameIDLoopCloure]);
  *latestSurfKeyFrameCloudBaseLinkFrame += (*surfCloudKeyFrames[latestFrameIDLoopCloure]);
  pcl::transformPointCloud(*latestSurfKeyFrameCloudBaseLinkFrame, *latestSurfKeyFrameCloudBaseLinkFrame, trans_b2c_af3_);
  //@ Narrow corrider should be prevented when doing icp: Narrow field of view usually introducing skewed icp result
  
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudBaseLinkFrame_Pass(new pcl::PointCloud<PointType>());
  auto original_size = latestSurfKeyFrameCloudBaseLinkFrame->points.size();
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (latestSurfKeyFrameCloudBaseLinkFrame);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-5.0, 5.0);
  pass.filter (*latestSurfKeyFrameCloudBaseLinkFrame_Pass);
  pass.setInputCloud (latestSurfKeyFrameCloudBaseLinkFrame_Pass);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-3.0, 3.0);
  pass.filter (*latestSurfKeyFrameCloudBaseLinkFrame_Pass);

  if( latestSurfKeyFrameCloudBaseLinkFrame_Pass->points.size() > original_size/2){
    RCLCPP_WARN(this->get_logger(), "Too clustered! In range: %lu, overall: %lu, ignore loop closure.", latestSurfKeyFrameCloudBaseLinkFrame_Pass->points.size(), original_size);
    return false;
  }

  // save history near key frames
  for (int j = - _history_keyframe_search_num; j <= _history_keyframe_search_num; ++j) {
    if (closestHistoryFrameID + j < 0 ||
        closestHistoryFrameID + j > latestFrameIDLoopCloure)
      continue;
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(
        cornerCloudKeyFrames[closestHistoryFrameID + j],
        &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(
        surfCloudKeyFrames[closestHistoryFrameID + j],
        &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
  }
  
  downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
  downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);
  // publish history near key frames
  
  if (true) {
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry_header_.stamp;
    cloudMsgTemp.header.frame_id = "camera_init";
    pubHistoryKeyFrames->publish(cloudMsgTemp);
  }
  
  return true;
}

void MapOptimization::performLoopClosure() {

  if (cloudKeyPoses3D->points.empty() == true)
    return;


  // try to find close key frame if there are any
  if (potentialLoopFlag == false) {
    if (detectLoopClosure() == true) {
      potentialLoopFlag = true;  // find some key frames that is old enough or
                                 // close enough for loop closure
      timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
    }
    if (potentialLoopFlag == false) return;
  }
  // reset the flag first no matter icp successes or not
  potentialLoopFlag = false;
  // ICP Settings
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  // Align clouds
  icp.setInputSource(latestSurfKeyFrameCloud);
  icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
  pcl::PointCloud<PointType>::Ptr unused_result(
      new pcl::PointCloud<PointType>());
  icp.align(*unused_result);
  RCLCPP_INFO(this->get_logger(), "ICP score: %.2f", icp.getFitnessScore());
  
  if (icp.hasConverged() == false ||
      icp.getFitnessScore() > _history_keyframe_fitness_score)
  {
    return;
  }
    
  // publish corrected cloud
  if (true) {
    pcl::PointCloud<PointType>::Ptr closed_cloud(
        new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*latestSurfKeyFrameCloud, *closed_cloud,
                             icp.getFinalTransformation());
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry_header_.stamp;
    cloudMsgTemp.header.frame_id = "camera_init";
    pubIcpKeyFrames->publish(cloudMsgTemp);
  }
  //
  //        get pose constraint
  //
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionCameraFrame;
  correctionCameraFrame =
      icp.getFinalTransformation();  // get transformation in camera frame
                                     // (because points are in camera frame)
  pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch,
                                    yaw);
  Eigen::Affine3f correctionLidarFrame =
      pcl::getTransformation(z, x, y, yaw, roll, pitch);
  // transform from world origin to wrong pose
  Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(
      cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  // transform from world origin to corrected pose
  Eigen::Affine3f tCorrect =
      correctionLidarFrame *
      tWrong;  // pre-multiplying -> successive rotation about a fixed frame
  pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
  gtsam::Pose3 poseFrom =
      Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  gtsam::Pose3 poseTo =
      pclPointTogtsamPose3(cloudKeyPoses6D->points[closestHistoryFrameID]);
  gtsam::Vector Vector6(6);

  float noiseScore = icp.getFitnessScore();
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
      noiseScore;
  constraintNoise = noiseModel::Diagonal::Variances(Vector6);
  //
  //        add constraints
  //
  std::pair<int, int> edge;
  edge.first = latestFrameIDLoopCloure;
  edge.second = closestHistoryFrameID;

  if(!pose_graph_.insert(edge).second)
  { 
    //@ We dont need to ignore the same edge, because the isam can keep update/interate using new edge relation
    //@ test and jestified on 05 July 2023
  }

  std::lock_guard<std::mutex> lock(mtx);
  gtSAMgraph.add(
      BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID,
                           poseFrom.between(poseTo), constraintNoise));
  isam->update(gtSAMgraph);
  isam->update();
  gtSAMgraph.resize(0);
  aLoopIsClosed = true;
}

void MapOptimization::extractSurroundingKeyFrames() {
  if (cloudKeyPoses3D->points.empty() == true) return;

  if (_loop_closure_enabled == true) {
    // only use recent key poses for graph building
    if (recentCornerCloudKeyFrames.size() <
        _surrounding_keyframe_search_num) {  // queue is not full (the beginning
                                         // of mapping or a loop is just
                                         // closed)
                                         // clear recent key frames queue
      recentCornerCloudKeyFrames.clear();
      recentSurfCloudKeyFrames.clear();
      recentOutlierCloudKeyFrames.clear();
      int numPoses = cloudKeyPoses3D->points.size();
      for (int i = numPoses - 1; i >= 0; --i) {
        int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
        PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
        updateTransformPointCloudSinCos(&thisTransformation);
        // extract surrounding map
        recentCornerCloudKeyFrames.push_front(
            transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
        recentSurfCloudKeyFrames.push_front(
            transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
        recentOutlierCloudKeyFrames.push_front(
            transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
        if (recentCornerCloudKeyFrames.size() >= _surrounding_keyframe_search_num)
          break;
      }
    } else {  // queue is full, pop the oldest key frame and push the latest
              // key frame
      if (latestFrameID != cloudKeyPoses3D->points.size() -
                               1) {  // if the robot is not moving, no need to
                                     // update recent frames

        recentCornerCloudKeyFrames.pop_front();
        recentSurfCloudKeyFrames.pop_front();
        recentOutlierCloudKeyFrames.pop_front();
        // push latest scan to the end of queue
        latestFrameID = cloudKeyPoses3D->points.size() - 1;
        PointTypePose thisTransformation =
            cloudKeyPoses6D->points[latestFrameID];
        updateTransformPointCloudSinCos(&thisTransformation);
        recentCornerCloudKeyFrames.push_back(
            transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
        recentSurfCloudKeyFrames.push_back(
            transformPointCloud(surfCloudKeyFrames[latestFrameID]));
        recentOutlierCloudKeyFrames.push_back(
            transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
      }
    }

    for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i) {
      *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
      *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
      *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i];
    }
  } else {
    surroundingKeyPoses->clear();
    surroundingKeyPosesDS->clear();
    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses.setInputCloud(cloudKeyPoses3D);
    kdtreeSurroundingKeyPoses.radiusSearch(
        currentRobotPosPoint, (double)_surrounding_keyframe_search_radius,
        pointSearchInd, pointSearchSqDis);

    for (int i = 0; i < pointSearchInd.size(); ++i){
      surroundingKeyPoses->points.push_back(
          cloudKeyPoses3D->points[pointSearchInd[i]]);
    }

    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

    // delete key frames that are not in surrounding region
    int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
    for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
      bool existingFlag = false;
      for (int j = 0; j < numSurroundingPosesDS; ++j) {
        if (surroundingExistingKeyPosesID[i] ==
            (int)surroundingKeyPosesDS->points[j].intensity) {
          existingFlag = true;
          break;
        }
      }
      if (existingFlag == false) {
        surroundingExistingKeyPosesID.erase(
            surroundingExistingKeyPosesID.begin() + i);
        surroundingCornerCloudKeyFrames.erase(
            surroundingCornerCloudKeyFrames.begin() + i);
        surroundingSurfCloudKeyFrames.erase(
            surroundingSurfCloudKeyFrames.begin() + i);
        surroundingOutlierCloudKeyFrames.erase(
            surroundingOutlierCloudKeyFrames.begin() + i);
        --i;
      }
    }
    // add new key frames that are not in calculated existing key frames
    for (int i = 0; i < numSurroundingPosesDS; ++i) {
      bool existingFlag = false;
      for (auto iter = surroundingExistingKeyPosesID.begin();
           iter != surroundingExistingKeyPosesID.end(); ++iter) {
        if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity) {
          existingFlag = true;
          break;
        }
      }
      if (existingFlag == true) {
        continue;
      } else {
        int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
        PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
        updateTransformPointCloudSinCos(&thisTransformation);
        surroundingExistingKeyPosesID.push_back(thisKeyInd);
        surroundingCornerCloudKeyFrames.push_back(
            transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
        surroundingSurfCloudKeyFrames.push_back(
            transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
        surroundingOutlierCloudKeyFrames.push_back(
            transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
      }
    }

    for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
      *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
      *laserCloudSurfFromMap += *surroundingSurfCloudKeyFrames[i];
      *laserCloudSurfFromMap += *surroundingOutlierCloudKeyFrames[i];
    }
  }
  // Downsample the surrounding corner key frames (or map)
  downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
  downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
  laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
  // Downsample the surrounding surf key frames (or map)
  downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
  downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
  laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
}

void MapOptimization::downsampleCurrentScan() {
  laserCloudCornerLastDS->clear();
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerLastDS);
  laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

  laserCloudSurfLastDS->clear();
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfLastDS);
  laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();

  laserCloudOutlierLastDS->clear();
  downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
  downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
  laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

  laserCloudSurfTotalLast->clear();
  laserCloudSurfTotalLastDS->clear();
  *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
  *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
  downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
  downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
  laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
}

void MapOptimization::cornerOptimization(int iterCount) {
  updatePointAssociateToMapSinCos();
  for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
    pointOri = laserCloudCornerLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd,
                                        pointSearchSqDis);

    if (pointSearchSqDis[4] < 1.0) {
      float cx = 0, cy = 0, cz = 0;
      for (int j = 0; j < 5; j++) {
        cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
        cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
        cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
      }
      cx /= 5;
      cy /= 5;
      cz /= 5;

      float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
      for (int j = 0; j < 5; j++) {
        float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
        float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
        float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      matA1(0, 0) = a11;
      matA1(0, 1) = a12;
      matA1(0, 2) = a13;
      matA1(1, 0) = a12;
      matA1(1, 1) = a22;
      matA1(1, 2) = a23;
      matA1(2, 0) = a13;
      matA1(2, 1) = a23;
      matA1(2, 2) = a33;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);

      matD1 = esolver.eigenvalues().real();
      matV1 = esolver.eigenvectors().real();

      if (matD1[2] > 3 * matD1[1]) {
        float x0 = pointSel.x;
        float y0 = pointSel.y;
        float z0 = pointSel.z;
        float x1 = cx + 0.1 * matV1(0, 0);
        float y1 = cy + 0.1 * matV1(0, 1);
        float z1 = cz + 0.1 * matV1(0, 2);
        float x2 = cx - 0.1 * matV1(0, 0);
        float y2 = cy - 0.1 * matV1(0, 1);
        float z2 = cz - 0.1 * matV1(0, 2);

        float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                              ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                          ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                              ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                          ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                              ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                         (z1 - z2) * (z1 - z2));

        float la =
            ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
             (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
            a012 / l12;

        float lb =
            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float lc =
            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float ld2 = a012 / l12;

        float s = 1 - 0.9 * fabs(ld2);

        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        if (s > 0.1) {
          laserCloudOri->push_back(pointOri);
          coeffSel->push_back(coeff);
        }
      }
    }
  }
}

void MapOptimization::surfOptimization(int iterCount) {
  updatePointAssociateToMapSinCos();
  for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
    pointOri = laserCloudSurfTotalLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd,
                                      pointSearchSqDis);

    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
        matA0(j, 1) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
        matA0(j, 2) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
      }
      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      float pa = matX0(0, 0);
      float pb = matX0(1, 0);
      float pc = matX0(2, 0);
      float pd = 1;

      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                 pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                 pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z +
                 pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1 - 0.9 * fabs(pd2) /
                          sqrt(sqrt(pointSel.x * pointSel.x +
                                    pointSel.y * pointSel.y +
                                    pointSel.z * pointSel.z));

        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        if (s > 0.1) {
          laserCloudOri->push_back(pointOri);
          coeffSel->push_back(coeff);
        }
      }
    }
  }
}

bool MapOptimization::LMOptimization(int iterCount) {
  float srx = sin(transformTobeMapped[0]);
  float crx = cos(transformTobeMapped[0]);
  float sry = sin(transformTobeMapped[1]);
  float cry = cos(transformTobeMapped[1]);
  float srz = sin(transformTobeMapped[2]);
  float crz = cos(transformTobeMapped[2]);

  int laserCloudSelNum = laserCloudOri->points.size();
  if (laserCloudSelNum < 50) {
    return false;
  }

  Eigen::Matrix<float,Eigen::Dynamic,6> matA(laserCloudSelNum, 6);
  Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,laserCloudSelNum);
  Eigen::Matrix<float,6,6> matAtA;
  Eigen::VectorXf matB(laserCloudSelNum);
  Eigen::Matrix<float,6,1> matAtB;
  Eigen::Matrix<float,6,1> matX;

  for (int i = 0; i < laserCloudSelNum; i++) {
    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float arx =
        (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
         srx * sry * pointOri.z) *
            coeff.x +
        (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) *
            coeff.y +
        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
         cry * srx * pointOri.z) *
            coeff.z;

    float ary =
        ((cry * srx * srz - crz * sry) * pointOri.x +
         (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) *
            coeff.x +
        ((-cry * crz - srx * sry * srz) * pointOri.x +
         (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) *
            coeff.z;

    float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                 (-cry * crz - srx * sry * srz) * pointOri.y) *
                    coeff.x +
                (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                ((sry * srz + cry * crz * srx) * pointOri.x +
                 (crz * sry - cry * srx * srz) * pointOri.y) *
                    coeff.z;

    matA(i, 0) = arx;
    matA(i, 1) = ary;
    matA(i, 2) = arz;
    matA(i, 3) = coeff.x;
    matA(i, 4) = coeff.y;
    matA(i, 5) = coeff.z;
    matB(i, 0) = -coeff.intensity;
  }
  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1,6> matE;
    Eigen::Matrix<float,6,6> matV;
    Eigen::Matrix<float,6,6> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();

     matV2 = matV;

    isDegenerate = false;
    float eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,6, 1> matX2(matX);
    matX2 = matX;
    matX = matP * matX2;
  }

  transformTobeMapped[0] += matX(0, 0);
  transformTobeMapped[1] += matX(1, 0);
  transformTobeMapped[2] += matX(2, 0);
  transformTobeMapped[3] += matX(3, 0);
  transformTobeMapped[4] += matX(4, 0);
  transformTobeMapped[5] += matX(5, 0);

  float deltaR = sqrt(pow(pcl::rad2deg(matX(0, 0)), 2) +
                      pow(pcl::rad2deg(matX(1, 0)), 2) +
                      pow(pcl::rad2deg(matX(2, 0)), 2));
  float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                      pow(matX(4, 0) * 100, 2) +
                      pow(matX(5, 0) * 100, 2));

  if (deltaR < 0.05 && deltaT < 0.05) {
    return true;
  }
  return false;
}

void MapOptimization::scan2MapOptimization() {
  if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {
    kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMapDS);
    kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMapDS);

    for (int iterCount = 0; iterCount < 10; iterCount++) {
      laserCloudOri->clear();
      coeffSel->clear();

      cornerOptimization(iterCount);
      surfOptimization(iterCount);

      if (LMOptimization(iterCount) == true) break;
    }

    transformUpdate();
  }
}

void MapOptimization::saveKeyFramesAndFactor() {
  currentRobotPos.x = transformAftMapped[3];
  currentRobotPos.y = transformAftMapped[4];
  currentRobotPos.z = transformAftMapped[5];
  currentRobotPos.yaw = transformAftMapped[1];
  currentRobotPosPoint.x = transformAftMapped[3];
  currentRobotPosPoint.y = transformAftMapped[4];
  currentRobotPosPoint.z = transformAftMapped[5];
  

  bool saveThisKeyFrame = true;
  if (sqrt((previousRobotPos.x - currentRobotPos.x) *
               (previousRobotPos.x - currentRobotPos.x) +
           (previousRobotPos.y - currentRobotPos.y) *
               (previousRobotPos.y - currentRobotPos.y) +
           (previousRobotPos.z - currentRobotPos.z) *
               (previousRobotPos.z - currentRobotPos.z)) < distance_between_key_frame_ &&
               fabs(previousRobotPos.yaw - currentRobotPos.yaw) < angle_between_key_frame_) {
    saveThisKeyFrame = false;
  }

  if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty()) return;

  previousRobotPos = currentRobotPos;
  //
  // update grsam graph
  //
  std::pair<int, int> edge;

  if (cloudKeyPoses3D->points.empty()) {

    edge.first = 0;
    edge.second = 0;

    gtSAMgraph.add(PriorFactor<Pose3>(
        0,
        Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                           transformTobeMapped[1]),
              Point3(transformTobeMapped[5], transformTobeMapped[3],
                     transformTobeMapped[4])),
        priorNoise));
    initialEstimate.insert(
        0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                              transformTobeMapped[1]),
                 Point3(transformTobeMapped[5], transformTobeMapped[3],
                        transformTobeMapped[4])));
    for (int i = 0; i < 6; ++i) transformLast[i] = transformTobeMapped[i];
  } else {

    edge.first = cloudKeyPoses3D->points.size() - 1;
    edge.second = cloudKeyPoses3D->points.size();

    //@ second pose should be computed and stored after optimization

    gtsam::Pose3 poseFrom = Pose3(
        Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
        Point3(transformLast[5], transformLast[3], transformLast[4]));
    gtsam::Pose3 poseTo =
        Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4]));
    gtSAMgraph.add(BetweenFactor<Pose3>(
        cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(),
        poseFrom.between(poseTo), odometryNoise));
    initialEstimate.insert(
        cloudKeyPoses3D->points.size(),
        Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4])));
  }
  //
  // update iSAM
  //
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  //
  // save key poses
  //
  PointType thisPose3D;
  PointTypePose thisPose6D;
  Pose3 latestEstimate;

  isamCurrentEstimate = isam->calculateEstimate();
  latestEstimate =
      isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

  thisPose3D.x = latestEstimate.translation().y();
  thisPose3D.y = latestEstimate.translation().z();
  thisPose3D.z = latestEstimate.translation().x();
  thisPose3D.intensity =
      cloudKeyPoses3D->points.size();  // this can be used as index
  cloudKeyPoses3D->push_back(thisPose3D);

  thisPose6D.x = thisPose3D.x;
  thisPose6D.y = thisPose3D.y;
  thisPose6D.z = thisPose3D.z;
  thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
  thisPose6D.roll = latestEstimate.rotation().pitch();
  thisPose6D.pitch = latestEstimate.rotation().yaw();
  thisPose6D.yaw = latestEstimate.rotation().roll();  // in camera frame
  thisPose6D.time = timeLaserOdometry;
  cloudKeyPoses6D->push_back(thisPose6D);

  //
  // save updated transform
  //
  if (cloudKeyPoses3D->points.size() > 1) {
    transformAftMapped[0] = latestEstimate.rotation().pitch();
    transformAftMapped[1] = latestEstimate.rotation().yaw();
    transformAftMapped[2] = latestEstimate.rotation().roll();
    transformAftMapped[3] = latestEstimate.translation().y();
    transformAftMapped[4] = latestEstimate.translation().z();
    transformAftMapped[5] = latestEstimate.translation().x();

    for (int i = 0; i < 6; ++i) {
      transformLast[i] = transformAftMapped[i];
      transformTobeMapped[i] = transformAftMapped[i];
    }
  }
  
  //@update graph
  if(!pose_graph_.insert(edge).second)
  {   
    RCLCPP_ERROR(this->get_logger(), "It is not possible! We are inserting the same edge in ISAM.");
  }

  
  pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisSurfFlatKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(
      new pcl::PointCloud<PointType>());

  pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
  pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
  pcl::copyPointCloud(*laserCloudSurfFlatLast, *thisSurfFlatKeyFrame);
  pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

  cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
  surfCloudKeyFrames.push_back(thisSurfKeyFrame);
  surfFlatCloudKeyFrames.push_back(thisSurfFlatKeyFrame);//for ground stitching
  outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
}

void MapOptimization::correctPoses() {
  if (aLoopIsClosed == true) {
    recentCornerCloudKeyFrames.clear();
    recentSurfCloudKeyFrames.clear();
    recentOutlierCloudKeyFrames.clear();
    // update key poses
    int numPoses = isamCurrentEstimate.size();
    for (int i = 0; i < numPoses; ++i) {
      cloudKeyPoses3D->points[i].x =
          isamCurrentEstimate.at<Pose3>(i).translation().y();
      cloudKeyPoses3D->points[i].y =
          isamCurrentEstimate.at<Pose3>(i).translation().z();
      cloudKeyPoses3D->points[i].z =
          isamCurrentEstimate.at<Pose3>(i).translation().x();

      cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
      cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
      cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
      cloudKeyPoses6D->points[i].roll =
          isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
      cloudKeyPoses6D->points[i].pitch =
          isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
      cloudKeyPoses6D->points[i].yaw =
          isamCurrentEstimate.at<Pose3>(i).rotation().roll();
    }
    aLoopIsClosed = false;
  }
}

void MapOptimization::clearCloud() {
  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  laserCloudCornerFromMapDS->clear();
  laserCloudSurfFromMapDS->clear();
}


void MapOptimization::run() {
  
  AssociationOut association;
  _input_channel.receive(association);

  std::lock_guard<std::mutex> lock(mtx);

  laserCloudCornerLast = association.cloud_corner_last;
  laserCloudSurfLast = association.cloud_surf_last;
  laserCloudSurfFlatLast = association.cloud_surf_flat_last;// for ground stitching
  laserCloudOutlierLast = association.cloud_outlier_last;

  timeLaserOdometry = association.laser_odometry.header.stamp.sec + association.laser_odometry.header.stamp.nanosec/1000000000.;
  timeLaserOdometry_header_.stamp = association.laser_odometry.header.stamp;
  

  OdometryToTransform(association.laser_odometry, transformSum);

  transformAssociateToMap();
  
  extractSurroundingKeyFrames();

  downsampleCurrentScan();

  scan2MapOptimization();
  
  saveKeyFramesAndFactor();
  
  correctPoses();
  
  publishTF();

  publishKeyPosesAndFrames();
  clearCloud();
  
}
