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

#include <boost/circular_buffer.hpp>
#include "imageProjection.h"

using std::placeholders::_1;

ImageProjection::ImageProjection(std::string name, Channel<ProjectionOut>& output_channel)
    : Node(name), _output_channel(output_channel)
{

  //supress the no intensity found log
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  /*
  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lslidar_point_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));
  */

  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lslidar_point_cloud", 2,
        std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));

  _pub_full_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("segmented_cloud_pure", 1);  

  _pub_full_info_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("full_cloud_info", 1);  

  _pub_ground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("ground_cloud", 1);  

  _pub_segmented_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("segmented_cloud", 1); 

  //@ this cloud is sent to perception 3d, therefore, we need good QoS
  _pub_segmented_cloud_pure = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("segmented_cloud_pure", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  _pub_segmented_cloud_info = this->create_publisher<cloud_msgs::msg::CloudInfo>
      ("segmented_cloud_info", 1); 

  _pub_outlier_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("outlier_cloud", 1); 
  
  declare_parameter("laser.num_vertical_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_vertical_scans", _vertical_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_vertical_scans: %d", _vertical_scans);
  
  declare_parameter("laser.num_horizontal_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_horizontal_scans", _horizontal_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_horizontal_scans: %d", _horizontal_scans);

  declare_parameter("laser.vertical_angle_bottom", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_bottom", _ang_bottom);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_bottom: %.2f", _ang_bottom);

  float vertical_angle_top;
  declare_parameter("laser.vertical_angle_top", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_top", vertical_angle_top);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_top: %.2f", vertical_angle_top);

  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);
  _ang_bottom = -( _ang_bottom - 0.1) * DEG_TO_RAD;
  _segment_alpha_X = _ang_resolution_X;
  _segment_alpha_Y = _ang_resolution_Y;
  
  declare_parameter("imageProjection.segment_theta", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.segment_theta", _segment_theta);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_theta: %.2f", _segment_theta);
  _segment_theta *= DEG_TO_RAD;
  
  declare_parameter("imageProjection.segment_valid_point_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.segment_valid_point_num", _segment_valid_point_num);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_valid_point_num: %d", _segment_valid_point_num);
  
  declare_parameter("imageProjection.segment_valid_line_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.segment_valid_line_num",_segment_valid_line_num);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_valid_line_num: %d", _segment_valid_line_num);

  declare_parameter("laser.ground_scan_index", rclcpp::ParameterValue(0));
  this->get_parameter("laser.ground_scan_index", _ground_scan_index);
  RCLCPP_INFO(this->get_logger(), "laser.ground_scan_index: %d", _ground_scan_index);

  declare_parameter("laser.sensor_mount_angle", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.sensor_mount_angle", _sensor_mount_angle);
  RCLCPP_INFO(this->get_logger(), "laser.sensor_mount_angle: %.2f", _sensor_mount_angle);

  _sensor_mount_angle *= DEG_TO_RAD;
  
  declare_parameter("imageProjection.maximum_detection_range", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.maximum_detection_range", _maximum_detection_range);
  RCLCPP_INFO(this->get_logger(), "imageProjection.maximum_detection_range: %.2f", _maximum_detection_range);

  const size_t cloud_size = _vertical_scans * _horizontal_scans;

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);
  
}


void ImageProjection::resetParameters() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  _laser_cloud_in->clear();
  _ground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);

  _seg_msg.start_ring_index.assign(_vertical_scans, 0);
  _seg_msg.end_ring_index.assign(_vertical_scans, 0);

  _seg_msg.segmented_cloud_ground_flag.assign(cloud_size, false);
  _seg_msg.segmented_cloud_col_ind.assign(cloud_size, 0);
  _seg_msg.segmented_cloud_range.assign(cloud_size, 0);
}


void ImageProjection::cloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
  // Reset parameters
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  _seg_msg.header = laserCloudMsg->header;

  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}


void ImageProjection::projectPointCloud() {
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];

    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    // find the row and column index in the image for this point
    float verticalAngle = std::asin(thisPoint.z / range);
        //std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));

    int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
    if (rowIdn < 0 || rowIdn >= _vertical_scans) {
      continue;
    }

    float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);

    int columnIdn = -round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans * 0.5;

    if (columnIdn >= _horizontal_scans){
      columnIdn -= _horizontal_scans;
    }

    if (columnIdn < 0 || columnIdn >= _horizontal_scans){
      continue;
    }

    if (range < 0.1 || range>_maximum_detection_range){
      continue;
    }

    _range_mat(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    size_t index = columnIdn + rowIdn * _horizontal_scans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }
}

void ImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();
  _seg_msg.start_orientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();
  _seg_msg.end_orientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  if (_seg_msg.end_orientation - _seg_msg.start_orientation > 3 * M_PI) {
    _seg_msg.end_orientation -= 2 * M_PI;
  } else if (_seg_msg.end_orientation - _seg_msg.start_orientation < M_PI) {
    _seg_msg.end_orientation += 2 * M_PI;
  }
  _seg_msg.orientation_diff =
      _seg_msg.end_orientation - _seg_msg.start_orientation;
}

void ImageProjection::groundRemoval() {
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < _horizontal_scans; ++j) {
    for (size_t i = 0; i < _ground_scan_index; ++i) {
      size_t lowerInd = j + (i)*_horizontal_scans;
      size_t upperInd = j + (i + 1) * _horizontal_scans;

      if (_full_cloud->points[lowerInd].intensity == -1 ||
          _full_cloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      float dX =
          _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      float dY =
          _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      float dZ =
          _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change

      if ( (vertical_angle - _sensor_mount_angle) <= 10 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
      }
    }
  }
  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }

  for (size_t i = 0; i <= _ground_scan_index; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1)
        _ground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
    }
  }
}

void ImageProjection::cloudSegmentation() {
  // segmentation process
  for (size_t i = 0; i < _vertical_scans; ++i)
    for (size_t j = 0; j < _horizontal_scans; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < _vertical_scans; ++i) {
    _seg_msg.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {
          if (i > _ground_scan_index && j % 5 == 0) {
            _outlier_cloud->push_back(
                _full_cloud->points[j + i * _horizontal_scans]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _horizontal_scans - 5) continue;
        }
        // mark ground points so they will not be considered as edge features
        // later
        _seg_msg.segmented_cloud_ground_flag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);
        // mark the points' column index for marking occlusion later
        _seg_msg.segmented_cloud_col_ind[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmented_cloud_range[sizeOfSegCloud] =
            _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    _seg_msg.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * _horizontal_scans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

void ImageProjection::labelComponents(int row, int col) {

  const float segmentThetaThreshold = tan(_segment_theta);

  std::vector<bool> lineCountFlag(_vertical_scans, false);
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= _vertical_scans){
        continue;
      }
      // at range image margin (left or right side)
      if (thisIndY < 0){
        thisIndY = _horizontal_scans - 1;
      }
      if (thisIndY >= _horizontal_scans){
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }
  else if (all_pushed.size() >= _segment_valid_point_num) {
    int lineCount = 0;
    for (size_t i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++_label_count;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void ImageProjection::publishClouds() {
  const auto& cloudHeader = _seg_msg.header;

  sensor_msgs::msg::PointCloud2 laserCloudTemp;

  auto PublishCloud = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (true) {
      pcl::toROSMsg(*cloud, laserCloudTemp);
      laserCloudTemp.header = cloudHeader;
      //laserCloudTemp.header.frame_id = "laser_link";
      pub->publish(laserCloudTemp);
    }
  };

  //PublishCloud(_pub_outlier_cloud, _outlier_cloud);
  //PublishCloud(_pub_segmented_cloud, _segmented_cloud);
  //PublishCloud(_pub_full_cloud, _full_cloud);
  //PublishCloud(_pub_ground_cloud, _ground_cloud);
  PublishCloud(_pub_segmented_cloud_pure, _segmented_cloud_pure);
  //PublishCloud(_pub_full_info_cloud, _full_info_cloud);

  if (_pub_segmented_cloud_info->get_subscription_count() != 0) {
    _pub_segmented_cloud_info->publish(_seg_msg);
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());

  std::swap( out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);

  _output_channel.send( std::move(out) );

}


