#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
//@kdtree and normals
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
 
#include <vector>
#include <list>
#include <unordered_map>
#include <cmath>
#include <thread>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

typedef pcl::PointXYZI  PointType;

const double DEG_TO_RAD = M_PI / 180.0;

struct LidarSensor{
    int vertical_scans;
    int horizontal_scans;
    float vertical_angle_bottom, vertical_angle_top;
    float horizontal_ang_resolution, vertical_ang_resolution;
    float pitch_angle;
};

class Pose6DOF{
  public:
    void setXYZRPY(float x, float y, float z, float roll, float pitch, float yaw);
    float x, y, z;
    float roll, pitch, yaw;
};

void Pose6DOF::setXYZRPY(float mx, float my, float mz, float mroll, float mpitch, float myaw){
  x=mx; y=my; z=mz; roll=mroll; pitch=mpitch; yaw=myaw;
}

struct ProjectionOut
{
  LidarSensor lidar_sensor;
  std::unordered_map<size_t, size_t> laser_cloud_raw_feature_index;
  std::unordered_map<size_t, size_t> laser_cloud_raw_horizontal_plane_index;
  pcl::PointCloud<PointType>::Ptr laser_cloud_raw_feature;
  pcl::PointCloud<PointType>::Ptr laser_cloud_raw_horizontal_plane;
  std::shared_ptr<cv::Mat> range_image;
};


struct AssociationOut
{
  pcl::PointCloud<PointType>::Ptr normal_feature_cloud;
  pcl::PointCloud<PointType>::Ptr plane_feature_cloud;
  nav_msgs::msg::Odometry feature_odometry;
};



#endif
