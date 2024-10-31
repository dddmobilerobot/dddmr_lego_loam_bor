#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "utility.h"
#include "channel.h"
#include "nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

class FeatureAssociation : public rclcpp::Node 
{

 public:
  FeatureAssociation(std::string name, Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);

  ~FeatureAssociation();

  void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomIn);
  void runFeatureAssociation();
  
  bool systemInitedLM;
  nav_msgs::msg::Odometry mappingOdometry;
  
 private:

  Channel<ProjectionOut>& _input_channel;
  Channel<AssociationOut>& _output_channel;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsLessSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsLessFlat;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOriginizedWheelOdometryPath;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOriginizedLaserOdometryPath;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_corner_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_surf_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloudLast;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;

  int _vertical_scans;
  int _horizontal_scans;
  float _scan_period;
  float _edge_threshold;
  float _surf_threshold;
  float _nearest_feature_dist_sqr;
  std::string odom_type_;
  std::string robot_frame_, sensor_frame_;
  bool first_odom_prepared_;
  tf2::Transform tf2_trans_b2s_, tf2_first_odom_inverse_, tf2_first_odom_;
  bool to_map_optimization_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex _odom_mutex; //look like one thread is not enough, we use mutually cb and lock it
  rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsFlatDense;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;
  pcl::VoxelGrid<PointType> downSizesSurfPointsFlatDense;
  
  double timeScanCur;

  cloud_msgs::msg::CloudInfo segInfo;
  std_msgs::msg::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  int skipFrameNum;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<int> pointSelCornerInd;
  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<int> pointSelSurfInd;
  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6];
  float transformLaserOdometrySum[6];
  float transformWheelOdometrySum[6];

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped laserOdometryTrans;

  bool isDegenerate;

  int frameCount;

 private:  

  void initializationValue();
  void adjustDistortion();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);

  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);

  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);

  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void checkSystemInitialization();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometryPath();

  void adjustOutlierCloud();
  void publishCloudsLast();
  
  void assignMappingOdometry(float (&ts)[6]);
  nav_msgs::msg::Path laser_odom_path_, wheel_odom_path_;

};

#endif // FEATUREASSOCIATION_H
