#ifndef POSEGRAPHMERGINGEDITOR_H
#define POSEGRAPHMERGINGEDITOR_H

#include "pose_graph_editor.h"

using namespace std::placeholders;

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

using namespace gtsam;

class PoseGraphMergeEditor : public rclcpp::Node 
{

typedef pcl::PointCloud<PointTypePose> point_cloud_t;
typedef std::set<std::pair<int, int>> edge_t;
public:

  PoseGraphMergeEditor(std::string name);
  ~PoseGraphMergeEditor();
  
  std::vector<std::shared_ptr<PoseGraphEditor>> pge_vector_;

private:
  
  rclcpp::Clock::SharedPtr clock_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_graph_dir_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merge_first_key_frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merge_second_key_frame_pub_;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node_selection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_focus_pg_sub_;
  void operationCommandCB(const std_msgs::msg::String::SharedPtr msg);
  void nodeSelectionCB(const std_msgs::msg::String::SharedPtr msg);
  void currentFocusCB(const std_msgs::msg::String::SharedPtr msg);

  std::string current_focus_pg_;
  pcl::PointCloud<PointType>::Ptr merge_first_key_frame_pointcloud_;
  pcl::PointCloud<PointType>::Ptr merge_second_key_frame_pointcloud_;
  pcl::PointCloud<PointType>::Ptr second_frame_pointcloud_icp_;
  
  Eigen::Affine3d first_frame_2_second_frame_af3d_;
  std::pair<int, int> merged_edge_;
};

#endif // POSEGRAPHMERGINGEDITOR_H
