#include "map_optimization.h"

using namespace gtsam;

MapOptimization::MapOptimization(std::string name,
                                 Channel<AssociationOut> &input_channel)
    : Node(name), input_channel_(input_channel), is_first_keyframe_(true)
{
  clock_ = this->get_clock();
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam_ = std::make_shared<gtsam::ISAM2>(parameters);
  
  //@initialize pointcloud revelant
  ds_pub_map_.setLeafSize(0.2, 0.2, 0.2);
  
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
}

void MapOptimization::extractSurroundingKeyFrames(){

  if(KF_.key_points_.empty())
    return;

  // only use recent key poses for graph building
  current_keyframe_stitch_at_map_frame_.reset(new pcl::PointCloud<PointType>());
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
    *current_keyframe_stitch_at_map_frame_+=*a_normal_feature_at_map_frame;
    *current_keyframe_stitch_at_map_frame_+=*a_plane_feature_at_map_frame;
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
    tf2_map2keyframe_pose_ = tf2_map2sensor_;
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

  if(is_first_keyframe_){
    saveKeyFramesAndFactor();
    return;
  }
  updateMap2SensorTF2();
  extractSurroundingKeyFrames();
  saveKeyFramesAndFactor();
}
