#include "map_optimization.h"

using namespace gtsam;

MapOptimization::MapOptimization(std::string name,
                                 Channel<AssociationOut> &input_channel)
    : Node(name), input_channel_(input_channel), is_first_keyframe_(true)
{

  clock_ = this->get_clock();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam_ = std::make_shared<gtsam::ISAM2>(parameters);
  
  //@initialize pointcloud revelant
  ds_pub_map_.setLeafSize(0.5, 0.5, 0.5);
  ds_current_keyframe_stitch_at_map_frame_.setLeafSize(0.2, 0.2, 0.2);
  ds_current_feature_frame_.setLeafSize(0.2, 0.2, 0.2);
  //@ ros interface
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);  

  timer_run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_pub_gbl_map_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_run_ = this->create_wall_timer(1ms, std::bind(&MapOptimization::run, this), timer_run_cb_group_);
  timer_pub_gbl_map_ = this->create_wall_timer(200ms, std::bind(&MapOptimization::publishGlobalMapThread, this), timer_pub_gbl_map_cb_group_);
  
}

MapOptimization::~MapOptimization()
{
  input_channel_.send({});
}

void MapOptimization::initializeMap2KeyframePose(){
  tf2_map2keyframe_pose_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2_map2keyframe_pose_.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
}

void MapOptimization::updateMap2SensorTF2(){
  //@ use tf2_last_keyframe_pose_ and odometry between feature_odometry_ and feature_odometry_binding_keyframe_ to infer current pose
  tf2::Transform tf2_odom2sensor;
  tf2_odom2sensor.setOrigin(tf2::Vector3(feature_odometry_.pose.pose.position.x, feature_odometry_.pose.pose.position.y, feature_odometry_.pose.pose.position.z));
  tf2_odom2sensor.setRotation(tf2::Quaternion(feature_odometry_.pose.pose.orientation.x, feature_odometry_.pose.pose.orientation.y, feature_odometry_.pose.pose.orientation.z, feature_odometry_.pose.pose.orientation.w));

  tf2::Transform tf2_odom2sensor_binding_keyframe;
  tf2_odom2sensor_binding_keyframe.setOrigin(tf2::Vector3(KF_.feature_odometry_binding_keyframe_.pose.pose.position.x, KF_.feature_odometry_binding_keyframe_.pose.pose.position.y, KF_.feature_odometry_binding_keyframe_.pose.pose.position.z));
  tf2_odom2sensor_binding_keyframe.setRotation(tf2::Quaternion(KF_.feature_odometry_binding_keyframe_.pose.pose.orientation.x, KF_.feature_odometry_binding_keyframe_.pose.pose.orientation.y, KF_.feature_odometry_binding_keyframe_.pose.pose.orientation.z, KF_.feature_odometry_binding_keyframe_.pose.pose.orientation.w));
  
  tf2_current_keyframe2sensor_.mult(tf2_odom2sensor_binding_keyframe.inverse(), tf2_odom2sensor);

  tf2_map2sensor_.mult(tf2_map2keyframe_pose_, tf2_current_keyframe2sensor_);
  tf2_map2sensor_ros_.transform.translation.x = tf2_map2sensor_.getOrigin().x();
  tf2_map2sensor_ros_.transform.translation.y = tf2_map2sensor_.getOrigin().y();
  tf2_map2sensor_ros_.transform.translation.z = tf2_map2sensor_.getOrigin().z();
  tf2_map2sensor_ros_.transform.rotation.x = tf2_map2sensor_.getRotation().x();
  tf2_map2sensor_ros_.transform.rotation.y = tf2_map2sensor_.getRotation().y();
  tf2_map2sensor_ros_.transform.rotation.z = tf2_map2sensor_.getRotation().z();
  tf2_map2sensor_ros_.transform.rotation.w = tf2_map2sensor_.getRotation().w();

  //@ calculate map2odom to complete tf tree
  tf2::Transform tf2_map2odom;
  tf2_map2odom.mult(tf2_map2sensor_, tf2_odom2sensor.inverse());
  geometry_msgs::msg::TransformStamped m2o;
  m2o.header.frame_id = "map";
  m2o.header.stamp = clock_->now();
  m2o.child_frame_id = feature_odometry_.header.frame_id;
  m2o.transform.translation.x = tf2_map2odom.getOrigin().x();
  m2o.transform.translation.y = tf2_map2odom.getOrigin().y();
  m2o.transform.translation.z = tf2_map2odom.getOrigin().z();
  m2o.transform.rotation.x = tf2_map2odom.getRotation().x();
  m2o.transform.rotation.y = tf2_map2odom.getRotation().y();
  m2o.transform.rotation.z = tf2_map2odom.getRotation().z();
  m2o.transform.rotation.w = tf2_map2odom.getRotation().w();
  tf_broadcaster_->sendTransform(m2o);
}

void MapOptimization::extractSurroundingKeyFrames(){

  if(KF_.key_points_.empty())
    return;

  // only use recent key poses for graph building
  current_normal_stitch_at_map_frame_.reset(new pcl::PointCloud<PointType>());
  current_plane_stitch_at_map_frame_.reset(new pcl::PointCloud<PointType>());
  int surrounding_keyframe_search_num = 5;
  int end_index = 0;
  if (KF_.key_points_.size() < surrounding_keyframe_search_num)  
    end_index = 0;
  else
    end_index = KF_.key_points_.size()-surrounding_keyframe_search_num;

  for (int i = KF_.key_points_.size()-1; i>=end_index; i--) {
    //RCLCPP_INFO(this->get_logger(), "----%d", i);
    //@ remember every keyframe is sensor_frame, we need to convert it to map frame
    Eigen::Affine3d trans_m2s_af3 = tf2::transformToEigen(KF_.key_poses_ros_[i]);
    //@transform to map frame
    pcl::PointCloud<PointType>::Ptr a_normal_feature_at_map_frame(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*KF_.normal_feature_keyframes_[i], *a_normal_feature_at_map_frame, trans_m2s_af3);
    pcl::PointCloud<PointType>::Ptr a_plane_feature_at_map_frame(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*KF_.plane_feature_keyframes_[i], *a_plane_feature_at_map_frame, trans_m2s_af3);
    *current_normal_stitch_at_map_frame_+=*a_normal_feature_at_map_frame;
    *current_plane_stitch_at_map_frame_+=*a_plane_feature_at_map_frame;
  }

}
void MapOptimization::saveKeyFramesAndFactor(){

  if(is_first_keyframe_){

    //@ calculate map2keyframe
    initializeMap2KeyframePose();
    //@ push back first keypose/feature frame/plane frame
    KF_.saveKeyFrames(normal_feature_cloud_, plane_feature_cloud_, tf2_map2keyframe_pose_, feature_odometry_);
    is_first_keyframe_ = false;
  }
  else{
    double distance_between_key_frame = 1.0;
    double angle_between_key_frame = 0.3;
    double dr, dp, dyaw;
    tf2::Matrix3x3 m(tf2_current_keyframe2sensor_.getRotation());
    m.getRPY(dr, dp, dyaw);
    if(sqrt(tf2_current_keyframe2sensor_.getOrigin().x()*tf2_current_keyframe2sensor_.getOrigin().x()+
            tf2_current_keyframe2sensor_.getOrigin().y()*tf2_current_keyframe2sensor_.getOrigin().y()+
            tf2_current_keyframe2sensor_.getOrigin().z()*tf2_current_keyframe2sensor_.getOrigin().z()) < distance_between_key_frame 
            && fabs(dyaw) < angle_between_key_frame) {
      return;
    }
    //@ distance and angle have move enough for the next key frame
    
    ds_current_keyframe_stitch_at_map_frame_.setInputCloud(current_normal_stitch_at_map_frame_);
    ds_current_keyframe_stitch_at_map_frame_.filter(*current_normal_stitch_at_map_frame_);
    ds_current_keyframe_stitch_at_map_frame_.setInputCloud(current_plane_stitch_at_map_frame_);
    ds_current_keyframe_stitch_at_map_frame_.filter(*current_plane_stitch_at_map_frame_);
    ds_current_feature_frame_.setInputCloud(normal_feature_cloud_);
    ds_current_feature_frame_.filter(*normal_feature_cloud_);
    ds_current_feature_frame_.setInputCloud(plane_feature_cloud_);
    ds_current_feature_frame_.filter(*plane_feature_cloud_);
    
    //compute normal
    pcl::PointCloud<PointType>::Ptr cloud_source_opti_transformed_ptr;
    cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f T_predict, T_final_plane;
    Eigen::Affine3f trans_m2s_af3 = tf2::transformToEigen(tf2_map2sensor_ros_).cast<float>();;
    T_predict = trans_m2s_af3.matrix();
    
    //@ TODO: use wheel odometry as initial guess
    //T_predict << 1.0, 0.0, 0.0, 0.0,
    //            0.0, 1.0, 0.0, 0.0,
    //            0.0, 0.0, 1.0, 0.0,
    //            0.0, 0.0, 0.0, 1.0;
    //RCLCPP_INFO(this->get_logger(), "GUESS ---> XYZ: %.2f, %.2f, %.2f", tf2_map2sensor_.getOrigin().x(), tf2_map2sensor_.getOrigin().y(), tf2_map2sensor_.getOrigin().z());
    OptimizedICPGN icp_opti;
    icp_opti.SetTargetCloud(current_plane_stitch_at_map_frame_);
    icp_opti.SetTransformationEpsilon(1e-2);
    icp_opti.SetMaxIterations(20);
    icp_opti.SetMaxCorrespondDistance(2.0);
    icp_opti.Match(plane_feature_cloud_, T_predict, cloud_source_opti_transformed_ptr, T_final_plane);
    if (!icp_opti.HasConverged() || icp_opti.GetFitnessScore() > 3.0)
    {
      //@ TODO: calculate RT between end() and end()-2
      RCLCPP_INFO(this->get_logger(), "Plane optimization not converged: %.2f", icp_opti.GetFitnessScore());
    }
    float plane_x, plane_y, plane_z, plane_roll, plane_pitch, plane_yaw;
    Eigen::Affine3f relative_rt_plane;
    relative_rt_plane = T_final_plane;
    pcl::getTranslationAndEulerAngles(relative_rt_plane, plane_x, plane_y, plane_z, plane_roll, plane_pitch, plane_yaw);
    
    // create a new tf using z, roll, pitch from plane optimization
    geometry_msgs::msg::TransformStamped tf2_map2sensor_plane_opt;
    tf2_map2sensor_plane_opt = tf2_map2sensor_ros_;
    tf2::Quaternion tf2_quaternion(tf2_map2sensor_ros_.transform.rotation.x, tf2_map2sensor_ros_.transform.rotation.y, tf2_map2sensor_ros_.transform.rotation.z, tf2_map2sensor_ros_.transform.rotation.w);
    tf2::Matrix3x3 m2(tf2_quaternion);
    double roll, pitch, yaw;
    m2.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "Replace z: %.2f to %.2f", tf2_map2sensor_ros_.transform.translation.z, plane_z);
    tf2_map2sensor_plane_opt.transform.translation.z = plane_z;
    RCLCPP_INFO(this->get_logger(), "Replace roll: %.2f to %.2f, pitch: %.2f to %.2f", roll, plane_roll, pitch, plane_pitch);
    tf2::Quaternion q_plane_opt;
    q_plane_opt.setRPY(plane_roll, plane_pitch, yaw);
    tf2_map2sensor_plane_opt.transform.rotation.x = q_plane_opt.x();
    tf2_map2sensor_plane_opt.transform.rotation.y = q_plane_opt.y();
    tf2_map2sensor_plane_opt.transform.rotation.z = q_plane_opt.z();
    tf2_map2sensor_plane_opt.transform.rotation.w = q_plane_opt.w();
    Eigen::Affine3f trans_m2s_plane_opt_af3 = tf2::transformToEigen(tf2_map2sensor_plane_opt).cast<float>();;
    Eigen::Matrix4f T_predict_plane_opt = trans_m2s_plane_opt_af3.matrix();
    // ----- normal
    Eigen::Matrix4f T_final;
    icp_opti.SetTargetCloud(current_normal_stitch_at_map_frame_);
    icp_opti.SetTransformationEpsilon(1e-3);
    icp_opti.SetMaxIterations(50);
    icp_opti.SetMaxCorrespondDistance(5.0);
    icp_opti.Match(normal_feature_cloud_, T_predict_plane_opt, cloud_source_opti_transformed_ptr, T_final);
    if (!icp_opti.HasConverged() || icp_opti.GetFitnessScore() > 3.0)
    {
      //@ TODO: calculate RT between end() and end()-2
      RCLCPP_INFO(this->get_logger(), "Normal optimization not converged: %.2f", icp_opti.GetFitnessScore());
    }
    float normal_x, normal_y, normal_z, normal_roll, normal_pitch, normal_yaw;
    Eigen::Affine3f relative_rt_normal;
    relative_rt_normal = T_final;
    pcl::getTranslationAndEulerAngles(relative_rt_normal, normal_x, normal_y, normal_z, normal_roll, normal_pitch, normal_yaw);
    //RCLCPP_INFO(this->get_logger(), "RT ---> XYZ: %.2f, %.2f, %.2f, RPY: %.2f, %.2f, %.2f", x, y, z, roll, pitch, yaw);

    tf2::Quaternion q;
    q.setRPY(normal_roll, normal_pitch, normal_yaw);
    tf2_map2keyframe_pose_.setOrigin(tf2::Vector3(normal_x, normal_y, normal_z));
    tf2_map2keyframe_pose_.setRotation(q);
    KF_.saveKeyFrames(normal_feature_cloud_, plane_feature_cloud_, tf2_map2keyframe_pose_, feature_odometry_);
  }
}

void MapOptimization::publishGlobalMapThread()
{
  if (KF_.key_poses_.empty()) 
    return;
  
  pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());
  for(int i = 0; i < KF_.key_poses_.size(); i++){
    Eigen::Affine3d trans_m2s_af3 = tf2::transformToEigen(KF_.key_poses_ros_[i]);
    //@transform to map frame
    pcl::PointCloud<PointType>::Ptr a_normal_feature_at_map_frame(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*KF_.normal_feature_keyframes_[i], *a_normal_feature_at_map_frame, trans_m2s_af3);
    pcl::PointCloud<PointType>::Ptr a_plane_feature_at_map_frame(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*KF_.plane_feature_keyframes_[i], *a_plane_feature_at_map_frame, trans_m2s_af3);
    *map+=*a_normal_feature_at_map_frame;
    *map+=*a_plane_feature_at_map_frame;
  }
  // downsample visualized points
  ds_pub_map_.setInputCloud(map);
  ds_pub_map_.filter(*map);
  
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*map, msg);
  msg.header.stamp = clock_->now();
  msg.header.frame_id = "map";
  pub_map_->publish(msg);
}

void MapOptimization::run() {
  
  AssociationOut association;
  input_channel_.receive(association);

  normal_feature_cloud_ = association.normal_feature_cloud;
  plane_feature_cloud_ = association.plane_feature_cloud;  
  feature_odometry_ = association.feature_odometry;
  lidar_sensor_ = association.lidar_sensor;

  if(is_first_keyframe_){
    saveKeyFramesAndFactor();
    return;
  }
  updateMap2SensorTF2();
  extractSurroundingKeyFrames();
  saveKeyFramesAndFactor();
}
