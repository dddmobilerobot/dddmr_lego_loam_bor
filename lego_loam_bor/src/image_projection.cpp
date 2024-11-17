#include "image_projection.h"

using std::placeholders::_1;

ImageProjection::ImageProjection(std::string name, Channel<ProjectionOut>& output_channel)
    : Node(name), output_channel_(output_channel)
{
  //supress the no intensity found log
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  clock_ = this->get_clock();

  declare_parameter("laser.num_vertical_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_vertical_scans", vertical_scans_);
  RCLCPP_INFO(this->get_logger(), "laser.num_vertical_scans: %d", vertical_scans_);
  
  declare_parameter("laser.num_horizontal_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_horizontal_scans", horizontal_scans_);
  RCLCPP_INFO(this->get_logger(), "laser.num_horizontal_scans: %d", horizontal_scans_);
  
  // this value should be negative
  declare_parameter("laser.vertical_angle_bottom", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_bottom", vertical_angle_bottom_);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_bottom: %.2f", vertical_angle_bottom_);

  declare_parameter("laser.vertical_angle_top", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_top", vertical_angle_top_);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_top: %.2f", vertical_angle_top_);

  horizontal_ang_resolution_ = (M_PI*2) / (horizontal_scans_);
  vertical_ang_resolution_ = DEG_TO_RAD*(vertical_angle_top_ - vertical_angle_bottom_) / float(vertical_scans_-1);
  vertical_angle_bottom_ = -(vertical_angle_bottom_) * DEG_TO_RAD;
  
  //@ if pointing to the ground, it is negative
  declare_parameter("laser.pitch_angle", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.pitch_angle", lidar_sensor_.pitch_angle);
  RCLCPP_INFO(this->get_logger(), "laser.pitch_angle: %.2f", lidar_sensor_.pitch_angle);

  lidar_sensor_.vertical_scans = vertical_scans_;
  lidar_sensor_.horizontal_scans = horizontal_scans_;
  lidar_sensor_.vertical_angle_bottom = vertical_angle_bottom_;
  lidar_sensor_.vertical_angle_top = vertical_angle_top_;
  lidar_sensor_.horizontal_ang_resolution = horizontal_ang_resolution_;
  lidar_sensor_.vertical_ang_resolution = vertical_ang_resolution_;

  laser_cloud_raw_projected_.reset(new pcl::PointCloud<PointType>());
  laser_cloud_raw_projected_->points.resize(vertical_scans_ * horizontal_scans_);

  pub_projected_image_ =  this->create_publisher<sensor_msgs::msg::Image>("projected_image", 1);
  pub_pointcloud_feature_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_feature", 1);  
  pub_pointcloud_horizontal_plane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_horizontal_plane", 1);  
  
}

void ImageProjection::ptrInitialization(std::string frame_id){
  laser_cloud_raw_feature_index_.clear();
  laser_cloud_raw_horizontal_plane_index_.clear();
  laser_cloud_raw_.reset(new pcl::PointCloud<PointType>());
  laser_cloud_raw_feature_.reset(new pcl::PointCloud<PointType>());
  laser_cloud_raw_horizontal_plane_.reset(new pcl::PointCloud<PointType>());
  laser_cloud_raw_->header.frame_id = frame_id;
  laser_cloud_raw_feature_->header.frame_id = frame_id;
  laser_cloud_raw_horizontal_plane_->header.frame_id = frame_id;

}

void ImageProjection::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  
  ptrInitialization(msg->header.frame_id);
  // Copy and remove NAN points
  pcl::fromROSMsg(*msg, *laser_cloud_raw_);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laser_cloud_raw_, *laser_cloud_raw_, indices);
  projectPointCloud();
  horizontalPlaneRemoval();
  publishClouds();
  toFeatureAssociatiion();

}

void ImageProjection::projectPointCloud() {

  // range image projection
  cv::Mat projected_image(cv::Size(horizontal_scans_, vertical_scans_), CV_16UC1);
  projected_image.setTo(cv::Scalar(0));
  const size_t cloud_size = laser_cloud_raw_->points.size();

  for (size_t i = 0; i < cloud_size; ++i) {

    PointType a_pt = laser_cloud_raw_->points[i];
    float range = sqrt(a_pt.x * a_pt.x +
                       a_pt.y * a_pt.y +
                       a_pt.z * a_pt.z);

    // find the row and column index in the image for this point
    float vertical_angle = std::asin(a_pt.z / range);
    
    // find row index
    int row_index = (vertical_angle + vertical_angle_bottom_) / vertical_ang_resolution_;

    if (row_index < 0 || row_index >= vertical_scans_) {
      continue;
    }
    
    // find column index
    float horizon_angle = std::atan2(a_pt.x, a_pt.y);
    
    int column_index = -round((horizon_angle - M_PI_2) / horizontal_ang_resolution_) + horizontal_scans_ * 0.5;

    if (column_index >= horizontal_scans_){
      column_index -= horizontal_scans_;
    }

    if (column_index < 0 || column_index >= horizontal_scans_){
      continue;
    }

    projected_image.at<unsigned short>(row_index, column_index) = (unsigned short)range*100; // to center-meter
    size_t index = column_index + row_index * horizontal_scans_;
    laser_cloud_raw_projected_->points[index] = a_pt;
    laser_cloud_raw_projected_->points[index].intensity = range;
  }
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;
  msg = cv_bridge::CvImage(hdr, "mono16", projected_image).toImageMsg();
  pub_projected_image_->publish(*msg);
}

void ImageProjection::horizontalPlaneRemoval() {

  for (size_t j = 0; j < horizontal_scans_; ++j) {
    for (size_t i = 0; i < vertical_scans_-1; ++i) {
      size_t lower_index = j + (i)*horizontal_scans_;
      size_t upper_index = j + (i + 1) * horizontal_scans_;
      float dx =
          laser_cloud_raw_projected_->points[upper_index].x - laser_cloud_raw_projected_->points[lower_index].x;
      float dy =
          laser_cloud_raw_projected_->points[upper_index].y - laser_cloud_raw_projected_->points[lower_index].y;
      float dz =
          laser_cloud_raw_projected_->points[upper_index].z - laser_cloud_raw_projected_->points[lower_index].z;

      float vertical_angle = std::atan2(dz , sqrt(dx * dx + dy * dy + dz * dz));

      if(fabs(vertical_angle - lidar_sensor_.pitch_angle) <= 10 * DEG_TO_RAD) {
        size_t index_0 = j + i * horizontal_scans_;
        laser_cloud_raw_horizontal_plane_index_[index_0] = laser_cloud_raw_horizontal_plane_->points.size();
        laser_cloud_raw_horizontal_plane_->push_back(laser_cloud_raw_projected_->points[index_0]);
      }
      else{
        size_t index_0 = j + i * horizontal_scans_;
        laser_cloud_raw_feature_index_[index_0] = laser_cloud_raw_feature_->points.size();
        laser_cloud_raw_feature_->push_back(laser_cloud_raw_projected_->points[index_0]);
      }
    }
  }

}

void ImageProjection::publishClouds(){

  sensor_msgs::msg::PointCloud2 pc2_msg;

  auto PublishCloud = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    pcl::toROSMsg(*cloud, pc2_msg);
    pc2_msg.header.stamp = clock_->now();
    pub->publish(pc2_msg);
  };

  PublishCloud(pub_pointcloud_feature_, laser_cloud_raw_feature_);
  PublishCloud(pub_pointcloud_horizontal_plane_, laser_cloud_raw_horizontal_plane_);
}

void ImageProjection::toFeatureAssociatiion(){

  ProjectionOut out;
  out.laser_cloud_raw_feature.reset(new pcl::PointCloud<PointType>());
  out.laser_cloud_raw_horizontal_plane.reset(new pcl::PointCloud<PointType>());
  
  std::swap(out.laser_cloud_raw_feature, laser_cloud_raw_feature_);
  std::swap(out.laser_cloud_raw_horizontal_plane, laser_cloud_raw_horizontal_plane_);
  out.lidar_sensor = lidar_sensor_;
  out.laser_cloud_raw_feature_index = laser_cloud_raw_feature_index_;
  out.laser_cloud_raw_horizontal_plane_index = laser_cloud_raw_horizontal_plane_index_;
  output_channel_.send( std::move(out) );
}