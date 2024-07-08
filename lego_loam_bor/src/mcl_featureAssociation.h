#ifndef MCLFEATUREASSOCIATION_H
#define MCLFEATUREASSOCIATION_H

#include "utility.h"
#include "channel.h"
#include "nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

class MCLFeatureAssociation : public rclcpp::Node 
{

 public:
  MCLFeatureAssociation(Channel<ProjectionOut>& input_channel);

  ~MCLFeatureAssociation();

  void runFeatureAssociation();

 private:

  Channel<ProjectionOut>& _input_channel;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsLessSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsLessFlat;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_corner_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_surf_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloudLast;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
  
  size_t _cycle_count;

  int _vertical_scans;
  int _horizontal_scans;
  float _scan_period;
  float _edge_threshold;
  float _surf_threshold;
  float _nearest_feature_dist_sqr;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  
  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  cloud_msgs::msg::CloudInfo segInfo;
  std_msgs::msg::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  int skipFrameNum;
  bool systemInitedLM;

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
  float transformSum[6];

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::msg::Odometry laserOdometry;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped laserOdometryTrans;

  bool isDegenerate;

  int frameCount;

 private:
  void initializationValue();
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
  void updateInitialGuess();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometry();

  void adjustOutlierCloud();
  void publishCloudsLast();
  
};

#endif // FEATUREASSOCIATION_H
