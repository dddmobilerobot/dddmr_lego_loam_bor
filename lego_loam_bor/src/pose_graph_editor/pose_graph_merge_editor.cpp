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

#include "pose_graph_merge_editor.h"
#include <future>

using namespace gtsam;


PoseGraphMergeEditor::PoseGraphMergeEditor(std::string name) : 
  Node(name), current_focus_pg_("")
{
  pge_vector_.clear();
  clock_ = this->get_clock();
  
  merge_first_key_frame_pointcloud_.reset(new pcl::PointCloud<PointType>());
  merge_second_key_frame_pointcloud_.reset(new pcl::PointCloud<PointType>());
  
  pose_graph_dir_pub_ = this->create_publisher<std_msgs::msg::String>("pose_graph_dir", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  operation_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphMergeEditor::operationCommandCB, this, std::placeholders::_1));

  node_selection_sub_ = this->create_subscription<std_msgs::msg::String>(
      "node_selection_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphMergeEditor::nodeSelectionCB, this, std::placeholders::_1));

  current_focus_pg_sub_ = this->create_subscription<std_msgs::msg::String>(
      "current_focus_pg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphMergeEditor::currentFocusCB, this, std::placeholders::_1));

  merge_first_key_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merge_first_key_frame_pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  merge_second_key_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merge_second_key_frame_pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

}

void PoseGraphMergeEditor::currentFocusCB(const std_msgs::msg::String::SharedPtr msg){
  current_focus_pg_ = msg->data;
}

void PoseGraphMergeEditor::nodeSelectionCB(const std_msgs::msg::String::SharedPtr msg){

  //@ pg_0_node/88;pg_0_node/89;
  std::stringstream node_full_name(msg->data);
  std::string one_node_full_name;
  bool is_anchor = false;
  int selected_node = 0;
  while(std::getline(node_full_name, one_node_full_name, ';'))
  {

    if(one_node_full_name=="anchor"){
      is_anchor = true;
      continue;
    }

    std::stringstream one_node_full_name_stream(one_node_full_name);
    std::string node_number;
    std::vector<std::string> node_number_list;
    while(std::getline(one_node_full_name_stream, node_number, '/'))
    {
      node_number_list.push_back(node_number);
    }
    RCLCPP_DEBUG(this->get_logger().get_child("PGME"), "%s", node_number_list[1].c_str());

    selected_node = stoi(node_number_list[1]);
    break;

  }
  
  if(is_anchor){

    if(one_node_full_name.find(pge_vector_[0]->getName()) != std::string::npos){
      merge_first_key_frame_pointcloud_.reset(new pcl::PointCloud<PointType>());
      for(int i=selected_node-3; i<selected_node+4; i++){
        if(i<0 || i > pge_vector_[0]->cornerCloudKeyFrames_.size()-1)
          continue;
        *merge_first_key_frame_pointcloud_ += (*pge_vector_[0]->cornerCloudKeyFrames_[i]);
      }
      sensor_msgs::msg::PointCloud2 cloud1MsgTemp;
      pcl::toROSMsg(*merge_first_key_frame_pointcloud_, cloud1MsgTemp);
      cloud1MsgTemp.header.stamp = clock_->now();
      cloud1MsgTemp.header.frame_id = pge_vector_[0]->getName();
      merge_first_key_frame_pub_->publish(cloud1MsgTemp);
      merged_edge_.first = selected_node;
    }

    if(pge_vector_.size()>1 && one_node_full_name.find(pge_vector_[1]->getName()) != std::string::npos){
      merge_second_key_frame_pointcloud_.reset(new pcl::PointCloud<PointType>());
      for(int i=selected_node-3; i<selected_node+4; i++){
        if(i<0 || i > pge_vector_[0]->cornerCloudKeyFrames_.size()-1)
          continue;
        *merge_second_key_frame_pointcloud_ += (*pge_vector_[1]->cornerCloudKeyFrames_[i]);
      }
      sensor_msgs::msg::PointCloud2 cloud1MsgTemp;
      pcl::toROSMsg(*merge_second_key_frame_pointcloud_, cloud1MsgTemp);
      cloud1MsgTemp.header.stamp = clock_->now();
      cloud1MsgTemp.header.frame_id = pge_vector_[1]->getName();
      merge_second_key_frame_pub_->publish(cloud1MsgTemp);
      merged_edge_.second = selected_node + pge_vector_[0]->poses_.size();
    }
  }


}

void PoseGraphMergeEditor::operationCommandCB(const std_msgs::msg::String::SharedPtr msg){
  
  bool publish_icped_pc = false;

  //@-----Merge accept-----
  if (msg->data.find("merge_accept") != std::string::npos) {

    std::stringstream merge_and_dir_stream(msg->data);
    std::string unit_string;
    std::vector<std::string> unit_string_list;
    std::string dir_for_saving_merge_files;
    while(std::getline(merge_and_dir_stream, unit_string, ':'))
    {
      unit_string_list.push_back(unit_string); //unit_string_list[0] is "merge", unit_string_list[1] is dir
    }
    dir_for_saving_merge_files = unit_string_list[1];
    
    if(!std::filesystem::is_directory(dir_for_saving_merge_files)){
      RCLCPP_WARN(this->get_logger().get_child("PGME"), "Directoy is invalid: %s, the merging is not proceed.", dir_for_saving_merge_files.c_str());
      return;
    }

    point_cloud_t poses_;
    edge_t edges_;

    std::vector<pcl::PointCloud<PointType>> cornerCloudKeyFrames_baselink_;
    std::vector<pcl::PointCloud<PointType>> surfCloudKeyFrames_baselink_;
    
    //@Merge poses
    for(auto it=pge_vector_[0]->poses_.begin();it!=pge_vector_[0]->poses_.end();it++){
      poses_.push_back((*it));
    }
    for(auto it=pge_vector_[1]->poses_.begin();it!=pge_vector_[1]->poses_.end();it++){

      PointTypePose a_pose = (*it);

      //shift pose index fir second pose graph
      a_pose.intensity += pge_vector_[0]->poses_.size();

      //compute new pose by icp result
      geometry_msgs::msg::TransformStamped tf2_ros_first_frame_2_second_frame = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);
      tf2::Stamped<tf2::Transform> tf2_first_frame_2_second_frame;
      tf2::fromMsg(tf2_ros_first_frame_2_second_frame, tf2_first_frame_2_second_frame);
      RCLCPP_DEBUG(this->get_logger().get_child("PGME"), "ICP transform: %.2f, %.2f, %.2f", tf2_first_frame_2_second_frame.getOrigin().x(), tf2_first_frame_2_second_frame.getOrigin().y(), tf2_first_frame_2_second_frame.getOrigin().z());
      
      tf2::Stamped<tf2::Transform> tf2_second_frame_pose;
      tf2::Quaternion q;
      q.setRPY( a_pose.roll, a_pose.pitch, a_pose.yaw);
      tf2_second_frame_pose.setRotation(q);
      tf2_second_frame_pose.setOrigin(tf2::Vector3(a_pose.x, a_pose.y, a_pose.z));
      RCLCPP_DEBUG(this->get_logger().get_child("PGME"), "original second frame pose: %.2f, %.2f, %.2f", tf2_second_frame_pose.getOrigin().x(), tf2_second_frame_pose.getOrigin().y(), tf2_second_frame_pose.getOrigin().z());

      tf2::Transform tf2_icped_second_frame;
      tf2_icped_second_frame.mult(tf2_first_frame_2_second_frame, tf2_second_frame_pose); //@ here is very important, it is icp_trans*node_2 pose
      RCLCPP_DEBUG(this->get_logger().get_child("PGME"), "icped second frame pose: %.2f, %.2f, %.2f", tf2_icped_second_frame.getOrigin().x(), tf2_icped_second_frame.getOrigin().y(), tf2_icped_second_frame.getOrigin().z());

      tf2::Matrix3x3 m(tf2_icped_second_frame.getRotation());
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      a_pose.x = tf2_icped_second_frame.getOrigin().x();
      a_pose.y = tf2_icped_second_frame.getOrigin().y();
      a_pose.z = tf2_icped_second_frame.getOrigin().z();
      a_pose.roll = roll;
      a_pose.pitch = pitch;
      a_pose.yaw = yaw;

      poses_.push_back(a_pose);
    }    
    RCLCPP_INFO(this->get_logger().get_child("PGME"), "Poses are merged with total size: %lu", poses_.size());
    
    //@Merge edges
    for(auto it=pge_vector_[0]->edges_.begin();it!=pge_vector_[0]->edges_.end();it++){
      edges_.insert((*it));
    }
    for(auto it=pge_vector_[1]->edges_.begin();it!=pge_vector_[1]->edges_.end();it++){
      auto a_edge = (*it);

      a_edge.first += pge_vector_[0]->poses_.size();
      a_edge.second += pge_vector_[0]->poses_.size();
      edges_.insert(a_edge);
    }
    RCLCPP_INFO(this->get_logger().get_child("PGME"), "Edges are merged with total size: %lu", edges_.size());
    
    //@Merge corner key frames
    for(auto it=pge_vector_[0]->cornerCloudKeyFrames_baselink_.begin();it!=pge_vector_[0]->cornerCloudKeyFrames_baselink_.end();it++){
      cornerCloudKeyFrames_baselink_.push_back(*(*it));
    }
    for(auto it=pge_vector_[1]->cornerCloudKeyFrames_baselink_.begin();it!=pge_vector_[1]->cornerCloudKeyFrames_baselink_.end();it++){
      cornerCloudKeyFrames_baselink_.push_back(*(*it));
    }
    RCLCPP_INFO(this->get_logger().get_child("PGME"), "cornerCloudKeyFrames_baselink_ are merged with total size: %lu", cornerCloudKeyFrames_baselink_.size());

    assert (poses_.size()!=cornerCloudKeyFrames_baselink_.size());

    //@Merge surf key frames
    for(auto it=pge_vector_[0]->surfCloudKeyFrames_baselink_.begin();it!=pge_vector_[0]->surfCloudKeyFrames_baselink_.end();it++){
      surfCloudKeyFrames_baselink_.push_back(*(*it));
    }
    for(auto it=pge_vector_[1]->surfCloudKeyFrames_baselink_.begin();it!=pge_vector_[1]->surfCloudKeyFrames_baselink_.end();it++){
      surfCloudKeyFrames_baselink_.push_back(*(*it));
    }
    RCLCPP_INFO(this->get_logger().get_child("PGME"), "surfCloudKeyFrames_baselink_ are merged with total size: %lu", surfCloudKeyFrames_baselink_.size());

    assert (poses_.size()!=surfCloudKeyFrames_baselink_.size());

    //@-----exporrt map-----
    //Create sub folder
    std::string export_dir_string = dir_for_saving_merge_files + "/" + currentDateTime();
    std::filesystem::create_directory(export_dir_string);
    //@ -----Write poses----- 
    pcl::io::savePCDFileASCII(export_dir_string + "/poses.pcd", poses_);
    
    //@ -----Write graph-----
    pcl::PointCloud<pcl::PointXYZ> edges;
    int edge_number = 0;
    //insert 0,0
    pcl::PointXYZ pt;
    pt.x = edge_number;
    // @ the first vertex of pg
    pt.y = 0;
    pt.z = 0;
    edges.push_back(pt);
    edge_number++;
    for(auto it = edges_.begin(); it!=edges_.end(); it++){
      pcl::PointXYZ pt;
      pt.x = edge_number;
      pt.y = (*it).first;
      pt.z = (*it).second;
      edges.push_back(pt);
      edge_number++;
    }
    // @ the second vertex of pg
    pt.x = edge_number;
    pt.y = pge_vector_[0]->poses_.size();
    pt.z = pge_vector_[0]->poses_.size();
    edges.push_back(pt);
    edge_number++;
    //@ Add merge edge
    pt.x = edge_number;
    pt.y = merged_edge_.first;
    pt.z = merged_edge_.second;
    edges.push_back(pt);    
    pcl::io::savePCDFileASCII(export_dir_string + "/edges.pcd", edges);

    //@ -----Write pcd-----
    std::string pcd_dir = export_dir_string + "/pcd";
    std::filesystem::create_directory(pcd_dir);
    for (int i = 0; i < poses_.points.size(); ++i) {
      int thisKeyInd = (int)poses_.points[i].intensity;
      pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_feature.pcd", cornerCloudKeyFrames_baselink_[thisKeyInd]);
      pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_ground.pcd", surfCloudKeyFrames_baselink_[thisKeyInd]);
    }
    RCLCPP_INFO(this->get_logger().get_child("PGME"), "Maps are saved: %s", export_dir_string.c_str());

    std_msgs::msg::String tmp_str;
    tmp_str.data = export_dir_string;
    pose_graph_dir_pub_->publish(tmp_str);
  }

  //@-----Merge ICP-----
  if(msg->data=="merge_icp"){
    //@check if the frame is different
    if(merge_first_key_frame_pointcloud_->points.empty() || merge_second_key_frame_pointcloud_->points.empty()){
      RCLCPP_INFO(this->get_logger().get_child("PGME"), "Must select two key frames.");
      return;
    }
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(10);
    // Align clouds
    icp.setInputSource(merge_second_key_frame_pointcloud_);
    icp.setInputTarget(merge_first_key_frame_pointcloud_);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    //icp.hasConverged() == false
    //icp.getFitnessScore() > _history_keyframe_fitness_score)
    auto icp_score = icp.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP score: %.2f", icp_score);
    
    auto first_frame_2_second_frame_af3 = icp.getFinalTransformation();
    first_frame_2_second_frame_af3d_ = first_frame_2_second_frame_af3.cast<double>();
    second_frame_pointcloud_icp_.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*merge_second_key_frame_pointcloud_, *second_frame_pointcloud_icp_, first_frame_2_second_frame_af3d_);

    sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
    pcl::toROSMsg(*second_frame_pointcloud_icp_, icpedcloud2MsgTemp);
    icpedcloud2MsgTemp.header.stamp = clock_->now();
    icpedcloud2MsgTemp.header.frame_id = pge_vector_[1]->getName();
    merge_second_key_frame_pub_->publish(icpedcloud2MsgTemp);
    return;
  }

  if(msg->data == "merge_px+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);
    tf2_ros_node1_2_node2.transform.translation.x += 0.01;
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_px-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);
    tf2_ros_node1_2_node2.transform.translation.x -= 0.01;
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_py+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);
    tf2_ros_node1_2_node2.transform.translation.y += 0.01;
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_py-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);
    tf2_ros_node1_2_node2.transform.translation.y -= 0.01;
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_pz+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);
    tf2_ros_node1_2_node2.transform.translation.z += 0.01;
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_pz-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);
    tf2_ros_node1_2_node2.transform.translation.z -= 0.01;
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_roll+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    roll += 0.01;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_roll-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    roll -= 0.01;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_pitch+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    pitch += 0.01;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_pitch-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    pitch -= 0.01;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_yaw+"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    yaw += 0.01;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(msg->data == "merge_yaw-"){
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(first_frame_2_second_frame_af3d_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(tf2_ros_node1_2_node2.transform.rotation.x, 
                    tf2_ros_node1_2_node2.transform.rotation.y, 
                    tf2_ros_node1_2_node2.transform.rotation.z, 
                    tf2_ros_node1_2_node2.transform.rotation.w)).getRPY(roll, pitch, yaw);
    yaw -= 0.01;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2_ros_node1_2_node2.transform.rotation.x = q.x();
    tf2_ros_node1_2_node2.transform.rotation.y = q.y();
    tf2_ros_node1_2_node2.transform.rotation.z = q.z();
    tf2_ros_node1_2_node2.transform.rotation.w = q.w();
    first_frame_2_second_frame_af3d_ = tf2::transformToEigen(tf2_ros_node1_2_node2);
    publish_icped_pc = true;
  }

  if(publish_icped_pc){
    //@ The if statement is to prevent no icp has been done
    second_frame_pointcloud_icp_.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*merge_second_key_frame_pointcloud_, *second_frame_pointcloud_icp_, first_frame_2_second_frame_af3d_);

    sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
    pcl::toROSMsg(*second_frame_pointcloud_icp_, icpedcloud2MsgTemp);
    icpedcloud2MsgTemp.header.stamp = clock_->now();
    icpedcloud2MsgTemp.header.frame_id = pge_vector_[1]->getName();
    merge_second_key_frame_pub_->publish(icpedcloud2MsgTemp);
  }
}

PoseGraphMergeEditor::~PoseGraphMergeEditor()
{
  
}