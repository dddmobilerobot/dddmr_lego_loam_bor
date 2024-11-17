#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "utility.h"
#include "channel.h"

//optimized icp gaussian newton
#include "opt_icp_gn/optimized_ICP_GN.h"
#include "opt_icp_gn/common.h"

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
class LidarOdometry{
  public:
    
    LidarOdometry();
    bool isInitialized();
    nav_msgs::msg::Odometry getLidarOdometry();
    void accumulateTransform(double x, double y, double z, double roll, double pitch, double yaw);
    void setChildFrameAndClock(std::string child_frame_id, rclcpp::Clock::SharedPtr clock);

  private:
    bool is_initialized_;
    rclcpp::Clock::SharedPtr clock_;
    tf2::Transform tf2_accumulated_;
    nav_msgs::msg::Odometry lidar_odometry_;
};

LidarOdometry::LidarOdometry(): is_initialized_(false){}

bool LidarOdometry::isInitialized(){
  return is_initialized_;
}

nav_msgs::msg::Odometry LidarOdometry::getLidarOdometry(){
  return lidar_odometry_;
}

void LidarOdometry::setChildFrameAndClock(std::string child_frame_id, rclcpp::Clock::SharedPtr clock){
  clock_ = clock;
  lidar_odometry_.header.frame_id = "odom";
  lidar_odometry_.child_frame_id = child_frame_id;
  tf2_accumulated_.setRotation(tf2::Quaternion(0, 0, 0, 1));
  tf2_accumulated_.setOrigin(tf2::Vector3(0, 0, 0));
  is_initialized_ = true;
}

void LidarOdometry::accumulateTransform(double x, double y, double z, double roll, double pitch, double yaw){

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf2::Transform tf2_relative;
  tf2_relative.setRotation(q);
  tf2_relative.setOrigin(tf2::Vector3(x, y, z));

  tf2_accumulated_.mult(tf2_accumulated_, tf2_relative);
  lidar_odometry_.pose.pose.position.x = tf2_accumulated_.getOrigin().x();
  lidar_odometry_.pose.pose.position.y = tf2_accumulated_.getOrigin().y();
  lidar_odometry_.pose.pose.position.z = tf2_accumulated_.getOrigin().z();
  lidar_odometry_.pose.pose.orientation.x = tf2_accumulated_.getRotation().x();
  lidar_odometry_.pose.pose.orientation.y = tf2_accumulated_.getRotation().y();
  lidar_odometry_.pose.pose.orientation.z = tf2_accumulated_.getRotation().z();
  lidar_odometry_.pose.pose.orientation.w = tf2_accumulated_.getRotation().w();
}

class FeatureAssociation : public rclcpp::Node 
{

 public:
  FeatureAssociation(std::string name, Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);
  ~FeatureAssociation();

  void runFeatureAssociation();
  
 private:
  
  void findNormalFeatures();
  void findPlaneFeatures();
  void calculateRelativeTransformationNormal();
  void calculateRelativeTransformationPlane();
  void fuseRelativePoses();
  void publishOdometryTF();
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_laser_odometry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_feature_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_icped_feature_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_plane_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_icped_plane_;

  Channel<ProjectionOut>& input_channel_;
  Channel<AssociationOut>& output_channel_;

  pcl::PointCloud<PointType>::Ptr normal_feature_current_;
  pcl::PointCloud<PointType>::Ptr normal_feature_last_;
  pcl::PointCloud<PointType>::Ptr plane_feature_current_;
  pcl::PointCloud<PointType>::Ptr plane_feature_last_;
  std::unordered_map<size_t, size_t> laser_cloud_raw_feature_index_;
  std::unordered_map<size_t, size_t> laser_cloud_raw_horizontal_plane_index_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  
  std::vector<pcl::PointCloud<PointType>::Ptr> vector_laser_cloud_raw_feature_;
  std::vector<pcl::PointCloud<PointType>::Ptr> vector_laser_cloud_raw_horizontal_plane_;
  
  int skip_cnt_;
  LidarSensor lidar_sensor_;
  LidarOdometry lidar_odometry_;
  Pose6DOF feature_relative_pose_, plane_relative_pose_;
};

#endif // FEATUREASSOCIATION_H
