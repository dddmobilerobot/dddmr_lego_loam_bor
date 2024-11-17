#include "image_projection.h"
#include "feature_association.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

// ROS bag
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include "rclcpp/serialization.hpp"

//interactive
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class BagReader : public rclcpp::Node
{
  public:
    BagReader();
    std::string getBagFilePath(){return bag_file_dir_;}
    std::string getPointCloudTopic(){return point_cloud_topic_;}
    std::string getOdometryTopic(){return odometry_topic_;}
    void writeLog(std::string input_str){RCLCPP_INFO(this->get_logger(), "%s", input_str.c_str());}
    rclcpp::Time first_odom_stamp_, current_odom_stamp_;
    int skip_frame_;
    bool pause_mapping_;
    float icp_score_;
    float history_keyframe_search_radius_;

  private:

    std::string bag_file_dir_;
    std::string point_cloud_topic_;
    std::string odometry_topic_;
  
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pause_;
    void bagPauseCb(const std_msgs::msg::Bool::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_skip_frame_;
    void bagSkipFrameCb(const std_msgs::msg::Int32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_icp_score_;
    void bagICPScoreCb(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_history_keyframe_search_radius_;
    void bagHistoryKeyframeSearchRadiusCb(const std_msgs::msg::Float32::SharedPtr msg);
};

BagReader::BagReader():Node("bag_reader"), pause_mapping_(true){
  
  declare_parameter("bag_file_dir", rclcpp::ParameterValue(""));
  this->get_parameter("bag_file_dir", bag_file_dir_);
  RCLCPP_INFO(this->get_logger(), "bag_file_dir: %s", bag_file_dir_.c_str());

  declare_parameter("point_cloud_topic", rclcpp::ParameterValue(""));
  this->get_parameter("point_cloud_topic", point_cloud_topic_);
  RCLCPP_INFO(this->get_logger(), "point_cloud_topic: %s", point_cloud_topic_.c_str());

  declare_parameter("odometry_topic", rclcpp::ParameterValue(""));
  this->get_parameter("odometry_topic", odometry_topic_);
  RCLCPP_INFO(this->get_logger(), "odometry_topic: %s", odometry_topic_.c_str());

  declare_parameter("skip_frame", rclcpp::ParameterValue(2));
  this->get_parameter("skip_frame", skip_frame_);
  RCLCPP_INFO(this->get_logger(), "skip_frame: %d", skip_frame_);  

  sub_pause_ = this->create_subscription<std_msgs::msg::Bool>(
        "lego_loam_bag_pause", 1,
        std::bind(&BagReader::bagPauseCb, this, std::placeholders::_1));
  sub_skip_frame_ = this->create_subscription<std_msgs::msg::Int32>(
        "lego_loam_bag_skip_frame", 1,
        std::bind(&BagReader::bagSkipFrameCb, this, std::placeholders::_1));
  sub_icp_score_ = this->create_subscription<std_msgs::msg::Float32>(
        "lego_loam_bag_icp_score", 1,
        std::bind(&BagReader::bagICPScoreCb, this, std::placeholders::_1));
  sub_history_keyframe_search_radius_ = this->create_subscription<std_msgs::msg::Float32>(
        "lego_loam_bag_history_keyframe_search_radius", 1,
        std::bind(&BagReader::bagHistoryKeyframeSearchRadiusCb, this, std::placeholders::_1));
}

void BagReader::bagPauseCb(const std_msgs::msg::Bool::SharedPtr msg){
  pause_mapping_ = msg->data;
}
void BagReader::bagSkipFrameCb(const std_msgs::msg::Int32::SharedPtr msg){
  skip_frame_ = msg->data;
}
void BagReader::bagICPScoreCb(const std_msgs::msg::Float32::SharedPtr msg){
  icp_score_ = msg->data;
}
void BagReader::bagHistoryKeyframeSearchRadiusCb(const std_msgs::msg::Float32::SharedPtr msg){
  history_keyframe_search_radius_ = msg->data;
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjection>("lego_loam_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("lego_loam_fa", projection_out_channel, association_out_channel);
  auto BR = std::make_shared<BagReader>();
  
  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = BR->getBagFilePath();
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  while (rclcpp::ok() && reader.has_next())
  {
    if(BR->pause_mapping_){
      rclcpp::spin_some(BR);
      continue;
    }
    
    // serialized data
    auto serialized_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    auto topic = serialized_message->topic_name;
    if (topic == BR->getPointCloudTopic())
    {
      sensor_msgs::msg::PointCloud2 msg;
      auto serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
      serializer.deserialize_message(&extracted_serialized_msg, &msg);
      sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg;
      lidar_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      *lidar_msg = msg;
      IP->cloudHandler(lidar_msg);
      FA->runFeatureAssociation();
    }
    rclcpp::spin_some(BR); 
  }
  rclcpp::shutdown();

  return 0;
}


