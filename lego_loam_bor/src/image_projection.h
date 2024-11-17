#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "utility.h"
#include "channel.h"

class ImageProjection : public rclcpp::Node 
{
  public:

    ImageProjection(std::string name, Channel<ProjectionOut>& output_channel);

    ~ImageProjection() = default;
    
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  private:
    
    rclcpp::Clock::SharedPtr clock_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_raw_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_raw_projected_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_raw_feature_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_raw_horizontal_plane_;
    
    LidarSensor lidar_sensor_;
    Channel<ProjectionOut>& output_channel_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_projected_image_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_feature_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_horizontal_plane_;

    std::unordered_map<size_t, size_t> laser_cloud_raw_feature_index_;
    std::unordered_map<size_t, size_t> laser_cloud_raw_horizontal_plane_index_;
    int vertical_scans_, horizontal_scans_;
    float vertical_angle_top_, vertical_angle_bottom_;
    float horizontal_ang_resolution_, vertical_ang_resolution_;
    float mount_angle_;
    std::shared_ptr<cv::Mat> projected_image_;
    
    void ptrInitialization(std::string frame_id);
    void projectPointCloud();
    void horizontalPlaneRemoval();
    void publishClouds();
    void toFeatureAssociatiion();
};



#endif  // IMAGEPROJECTION_H
