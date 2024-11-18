#include "feature_association.h"

const float RAD2DEG = 180.0 / M_PI;

FeatureAssociation::FeatureAssociation(std::string name, Channel<ProjectionOut> &input_channel,
                                       Channel<AssociationOut> &output_channel) : 
Node(name), input_channel_(input_channel), output_channel_(output_channel), skip_cnt_(-1){

  clock_ = this->get_clock();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  ds_normal_feature_.setLeafSize(0.05, 0.05, 0.05);
  ds_plane_feature_.setLeafSize(0.05, 0.05, 0.05);
  
  pub_feature_image_ =  this->create_publisher<sensor_msgs::msg::Image>("feature_image", 1);
  pub_laser_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("laser_odometry", 1);
  pub_current_feature_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_current_feature", 1); 
  pub_icped_feature_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_icped_feature", 1); 
  pub_current_plane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_current_plane", 1); 
  pub_icped_plane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_icped_plane", 1);

  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
  timer_ = this->create_wall_timer(2ms, std::bind(&FeatureAssociation::runFeatureAssociation, this), timer_cb_group_);

}

FeatureAssociation::~FeatureAssociation()
{
  input_channel_.send({});
}

void FeatureAssociation::publishOdometryTF(){
  pub_laser_odometry_->publish(lidar_odometry_.getLidarOdometry());
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header = lidar_odometry_.getLidarOdometry().header;
  odom_tf.child_frame_id = lidar_odometry_.getLidarOdometry().child_frame_id;
  odom_tf.transform.rotation.x = lidar_odometry_.getLidarOdometry().pose.pose.orientation.x;
  odom_tf.transform.rotation.y = lidar_odometry_.getLidarOdometry().pose.pose.orientation.y;
  odom_tf.transform.rotation.z = lidar_odometry_.getLidarOdometry().pose.pose.orientation.z;
  odom_tf.transform.rotation.w = lidar_odometry_.getLidarOdometry().pose.pose.orientation.w;
  odom_tf.transform.translation.x = lidar_odometry_.getLidarOdometry().pose.pose.position.x;
  odom_tf.transform.translation.y = lidar_odometry_.getLidarOdometry().pose.pose.position.y;
  odom_tf.transform.translation.z = lidar_odometry_.getLidarOdometry().pose.pose.position.z;
  tf_broadcaster_->sendTransform(odom_tf);
}

void FeatureAssociation::calculateRelativeTransformationNormal(){
  
  if(vector_laser_cloud_raw_feature_.size()<2){
    //lidar_odometry_.accumulateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    feature_relative_pose_.setXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
  else{
    pcl::PointCloud<PointType>::Ptr normal_feature_current;
    pcl::PointCloud<PointType>::Ptr normal_feature_last;
    normal_feature_current = normal_feature_current_;
    normal_feature_last = normal_feature_last_;
    /*
    T_predict << 1.0, 0.0, 0.0, 'x',
                0.0, 1.0, 0.0, 'y',
                0.0, 0.0, 1.0, 'z',
                0.0, 0.0, 0.0, 1.0;  
    */
    pcl::PointCloud<PointType>::Ptr cloud_source_opti_transformed_ptr;
    cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f T_predict, T_final;
    T_predict.setIdentity();

    //@ TODO: use wheel odometry as initial guess
    T_predict << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
    OptimizedICPGN icp_opti;
    icp_opti.SetTargetCloud(normal_feature_last);
    icp_opti.SetTransformationEpsilon(1e-2);
    icp_opti.SetMaxIterations(20);
    icp_opti.SetMaxCorrespondDistance(1.0);
    icp_opti.Match(normal_feature_current, T_predict, cloud_source_opti_transformed_ptr, T_final);
    if (!icp_opti.HasConverged() || icp_opti.GetFitnessScore() > 3.0)
    {
      //@ TODO: calculate RT between end() and end()-2
      RCLCPP_INFO(this->get_logger(), "Normal relative pose not converged: %.2f", icp_opti.GetFitnessScore());
    }

    //RCLCPP_INFO_STREAM(this->get_logger(), "T_final: \n" << T_final);
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f relative_rt;
    relative_rt = T_final;
    pcl::getTranslationAndEulerAngles(relative_rt, x, y, z, roll, pitch, yaw);
    //RCLCPP_INFO(this->get_logger(), "RT ---> XYZ: %.2f, %.2f, %.2f, RPY: %.2f, %.2f, %.2f", x, y, z, roll, pitch, yaw);
    feature_relative_pose_.setXYZRPY(x, y, z, roll, pitch, yaw);

    pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*normal_feature_current, *closed_cloud, T_final);
    sensor_msgs::msg::PointCloud2 pc2_msg;
    pcl::toROSMsg(*closed_cloud, pc2_msg);
    pc2_msg.header.frame_id = normal_feature_current->header.frame_id;
    pub_icped_feature_->publish(pc2_msg);

    sensor_msgs::msg::PointCloud2 pc2_current_msg;
    pcl::toROSMsg(*normal_feature_last, pc2_current_msg);
    pc2_msg.header.frame_id = normal_feature_last->header.frame_id;
    pub_current_feature_->publish(pc2_current_msg);
    
  }
}

void FeatureAssociation::calculateRelativeTransformationPlane(){
  
  if(vector_laser_cloud_raw_horizontal_plane_.size()<2){
    //lidar_odometry_.accumulateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    plane_relative_pose_.setXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
  else{
    pcl::PointCloud<PointType>::Ptr plane_feature_current;
    pcl::PointCloud<PointType>::Ptr plane_feature_last;
    plane_feature_current = plane_feature_current_;
    plane_feature_last = plane_feature_last_;
    /*
    T_predict << 1.0, 0.0, 0.0, 'x',
                0.0, 1.0, 0.0, 'y',
                0.0, 0.0, 1.0, 'z',
                0.0, 0.0, 0.0, 1.0;  
    */
    pcl::PointCloud<PointType>::Ptr cloud_source_opti_transformed_ptr;
    cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f T_predict, T_final;
    T_predict.setIdentity();

    //@ TODO: use wheel odometry as initial guess
    T_predict << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
    OptimizedICPGN icp_opti;
    icp_opti.SetTargetCloud(plane_feature_last);
    icp_opti.SetTransformationEpsilon(1e-2);
    icp_opti.SetMaxIterations(20);
    icp_opti.SetMaxCorrespondDistance(1.0);
    icp_opti.Match(plane_feature_current, T_predict, cloud_source_opti_transformed_ptr, T_final);
    if (!icp_opti.HasConverged() || icp_opti.GetFitnessScore() > 3.0)
    {
      //@ TODO: calculate RT between end() and end()-2
      RCLCPP_INFO(this->get_logger(), "Plane relative pose not converged: %.2f", icp_opti.GetFitnessScore());
    }

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f relative_rt;
    relative_rt = T_final;
    pcl::getTranslationAndEulerAngles(relative_rt, x, y, z, roll, pitch, yaw);
    //RCLCPP_INFO(this->get_logger(), "RT ---> XYZ: %.2f, %.2f, %.2f, RPY: %.2f, %.2f, %.2f", x, y, z, roll, pitch, yaw);
    plane_relative_pose_.setXYZRPY(x, y, z, roll, pitch, yaw);

    pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*plane_feature_current, *closed_cloud, T_final);
    sensor_msgs::msg::PointCloud2 pc2_msg;
    pcl::toROSMsg(*closed_cloud, pc2_msg);
    pc2_msg.header.frame_id = plane_feature_current->header.frame_id;
    pub_icped_plane_->publish(pc2_msg);

    sensor_msgs::msg::PointCloud2 pc2_current_msg;
    pcl::toROSMsg(*plane_feature_last, pc2_current_msg);
    pc2_msg.header.frame_id = plane_feature_last->header.frame_id;
    pub_current_plane_->publish(pc2_current_msg);
    
  }
}

void FeatureAssociation::findNormalFeatures(){
  
  if(vector_laser_cloud_raw_feature_.size()>1){
    normal_feature_last_ = normal_feature_current_;
  }
  
  //see: https://learnopencv.com/edge-detection-using-opencv/
  cv::Mat vertical_feature_result;
  cv::Sobel(*range_image_, vertical_feature_result, CV_8UC1, 1, 0, 5);
  cv::Mat horizontal_feature_result;
  cv::Sobel(*range_image_, horizontal_feature_result, CV_8UC1, 0, 1, 3);

  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;
  msg = cv_bridge::CvImage(hdr, "mono8", vertical_feature_result).toImageMsg();
  pub_feature_image_->publish(*msg);

  normal_feature_current_.reset(new pcl::PointCloud<PointType>());
  normal_feature_current_->header = vector_laser_cloud_raw_feature_.back()->header;
  for (size_t i = 0; i < lidar_sensor_.vertical_scans; ++i ) {
    for (size_t j = 0; j < lidar_sensor_.horizontal_scans; ++j) {
      size_t index = j + (i)*lidar_sensor_.horizontal_scans;
      if(laser_cloud_raw_feature_index_.find(index) == laser_cloud_raw_feature_index_.end())
        continue;
      if(vertical_feature_result.at<unsigned char>(i, j)==255 || horizontal_feature_result.at<unsigned char>(i, j)==255 )
        normal_feature_current_->push_back(vector_laser_cloud_raw_feature_.back()->points[laser_cloud_raw_feature_index_[index]]);
    }
  }
  ds_normal_feature_.setInputCloud(normal_feature_current_);
  ds_normal_feature_.filter(*normal_feature_current_);
  //RCLCPP_INFO(this->get_logger(), "Features: %lu", normal_feature_current_->points.size());
}

void FeatureAssociation::findPlaneFeatures(){
  if(vector_laser_cloud_raw_horizontal_plane_.size()>1){
    plane_feature_last_ = plane_feature_current_;
  }
  plane_feature_current_.reset(new pcl::PointCloud<PointType>());
  plane_feature_current_->header = vector_laser_cloud_raw_horizontal_plane_.back()->header;
  for (size_t i = 0; i < lidar_sensor_.vertical_scans/2; ++i ) {
    for (size_t j = 1; j < lidar_sensor_.horizontal_scans; j++) {
      size_t index = j + (i)*lidar_sensor_.horizontal_scans;
      if(laser_cloud_raw_horizontal_plane_index_.find(index) == laser_cloud_raw_horizontal_plane_index_.end())
        continue;
      PointType a_pt = vector_laser_cloud_raw_horizontal_plane_.back()->points[laser_cloud_raw_horizontal_plane_index_[index]];
      if(sqrt(a_pt.x*a_pt.x + a_pt.y*a_pt.y + a_pt.z*a_pt.z)>5.0)
        continue;
      plane_feature_current_->push_back(a_pt);
    }
  }
  ds_plane_feature_.setInputCloud(plane_feature_current_);
  ds_plane_feature_.filter(*plane_feature_current_);
  //RCLCPP_INFO(this->get_logger(), "Plane: %lu", plane_feature_current_->points.size());
}

void FeatureAssociation::fuseRelativePoses(){
  lidar_odometry_.accumulateTransform(feature_relative_pose_.x, feature_relative_pose_.y, plane_relative_pose_.z, plane_relative_pose_.roll, plane_relative_pose_.pitch, feature_relative_pose_.yaw);
}

void FeatureAssociation::runFeatureAssociation(){

  ProjectionOut projection;
  input_channel_.receive(projection);
  lidar_sensor_ = projection.lidar_sensor;
  laser_cloud_raw_feature_index_ = projection.laser_cloud_raw_feature_index;
  laser_cloud_raw_horizontal_plane_index_ = projection.laser_cloud_raw_horizontal_plane_index;
  range_image_ = projection.range_image;

  //@ append observation to the vector
  if(vector_laser_cloud_raw_feature_.size()<2){
    vector_laser_cloud_raw_feature_.push_back(projection.laser_cloud_raw_feature);
    vector_laser_cloud_raw_horizontal_plane_.push_back(projection.laser_cloud_raw_horizontal_plane);
  }
  else{
    vector_laser_cloud_raw_feature_.erase(vector_laser_cloud_raw_feature_.begin());
    vector_laser_cloud_raw_horizontal_plane_.erase(vector_laser_cloud_raw_horizontal_plane_.begin());
    vector_laser_cloud_raw_feature_.push_back(projection.laser_cloud_raw_feature);
    vector_laser_cloud_raw_horizontal_plane_.push_back(projection.laser_cloud_raw_horizontal_plane);
  }
  
  if(!lidar_odometry_.isInitialized())
    lidar_odometry_.setChildFrameAndClock(projection.laser_cloud_raw_feature->header.frame_id, clock_);
  
  findNormalFeatures();
  findPlaneFeatures();
  calculateRelativeTransformationNormal();
  calculateRelativeTransformationPlane();
  fuseRelativePoses();
  publishOdometryTF();

  AssociationOut out;
  out.normal_feature_cloud = normal_feature_current_;
  out.plane_feature_cloud = plane_feature_current_;
  out.feature_odometry = lidar_odometry_.getLidarOdometry();
  output_channel_.send(std::move(out));

}
