#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"
#include "transformFusion.h"
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

  private:
    std::string bag_file_dir_;
    std::string point_cloud_topic_;
    std::string odometry_topic_;
    
};

BagReader::BagReader():Node("bag_reader"){

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
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjection>("lego_loam_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("lego_loam_fa", projection_out_channel, association_out_channel);
  auto MO = std::make_shared<MapOptimization>("lego_loam_mo", association_out_channel);
  auto TF = std::make_shared<TransformFusion>("lego_loam_tf");
  auto BR = std::make_shared<BagReader>();

  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = BR->getBagFilePath();
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  //@calculate clock
  double last_wall_time = 0.0;
  bool go_first_odom = false;
  struct timeval start, end;
  gettimeofday(&start, NULL);
  int cycle_cnt = 0;
  while (rclcpp::ok() && reader.has_next())
  {
    // serialized data
    auto serialized_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    auto topic = serialized_message->topic_name;

    if (topic == BR->getOdometryTopic())
    {
      nav_msgs::msg::Odometry msg;
      auto serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
      serializer.deserialize_message(&extracted_serialized_msg, &msg);
      nav_msgs::msg::Odometry::SharedPtr odom;
      odom = std::make_shared<nav_msgs::msg::Odometry>();
      *odom = msg;
      FA->odomHandler(odom);
      if(!go_first_odom){
        BR->first_odom_stamp_ = msg.header.stamp;
        BR->current_odom_stamp_ = msg.header.stamp;
        go_first_odom = true;
      }
      else{
        BR->current_odom_stamp_ = msg.header.stamp;
      }
    }

    if (topic == BR->getPointCloudTopic())
    {
      sensor_msgs::msg::PointCloud2 msg;
      auto serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
      serializer.deserialize_message(&extracted_serialized_msg, &msg);
      sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg;
      laserCloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      *laserCloudMsg = msg;
      IP->cloudHandler(laserCloudMsg);
      bool FA_ready = FA->systemInitedLM; //This line should come before FA->runFeatureAssociation()
      FA->runFeatureAssociation();

      nav_msgs::msg::Odometry::SharedPtr mapping_odom;
      mapping_odom = std::make_shared<nav_msgs::msg::Odometry>();
      *mapping_odom = FA->mappingOdometry;
      TF->laserOdometryHandler(mapping_odom);

      if(FA_ready && cycle_cnt%BR->skip_frame_==0){
        MO->run();
        nav_msgs::msg::Odometry::SharedPtr odom_aft_mapped;
        odom_aft_mapped = std::make_shared<nav_msgs::msg::Odometry>();
        *odom_aft_mapped = MO->odomAftMapped;
        TF->odomAftMappedHandler(odom_aft_mapped);
        MO->publishGlobalMapThread();
        MO->loopClosureThread();
      }
      cycle_cnt++;
    }
    //@ calculate bag time and wall ime
    double bag_time_diff = (BR->current_odom_stamp_ - BR->first_odom_stamp_).seconds();
    gettimeofday(&end, NULL);
    double start_t, end_t;
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    double wall_time_diff = end_t - start_t;
    double processing_speed = bag_time_diff/wall_time_diff;
    if(wall_time_diff - last_wall_time > 5.0){
      BR->writeLog("Processing Speed: " + std::to_string(processing_speed));
      last_wall_time = wall_time_diff;
    }
    
  }
  std::shared_ptr<std_srvs::srv::Empty::Request> request;
  std::shared_ptr<std_srvs::srv::Empty::Response> response;

  MO->pcdSaver(request, response);
  /*
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(IP);
  executor.add_node(FA);
  executor.add_node(MO);
  executor.add_node(TF);
  executor.spin();
  */
  rclcpp::shutdown();

  return 0;
}


