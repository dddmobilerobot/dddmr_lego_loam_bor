#include "pose_graph_editor.h"
#include "pose_graph_merge_editor.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <bits/stdc++.h>

using namespace std::chrono_literals;

class PoseGraphEditorNode : public rclcpp::Node
{
  public:

    PoseGraphEditorNode();

  private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pose_graph_dir_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_pose_graphs_pub_;
    void cbSub(const std_msgs::msg::String::SharedPtr msg);
    void spinPoseGraph(std::shared_ptr<PoseGraphEditor> p_ptr);
    void spinPoseGraphMerge(std::shared_ptr<PoseGraphMergeEditor> p_ptr);

    std::set<std::string> pose_graphs_; //used to store pose graph, to prevent duplicated pose graph
    std::map<int, std::string> all_pose_graphs_;
    int pose_graph_num_;

    //@all about merge
    std::shared_ptr<PoseGraphMergeEditor> pgme_;

};

PoseGraphEditorNode::PoseGraphEditorNode():Node("pose_graph_editor_node"), pose_graph_num_(0){
  
  pgme_ = std::make_shared<PoseGraphMergeEditor>("pgme");

  pose_graph_dir_sub_ = this->create_subscription<std_msgs::msg::String>(
      "pose_graph_dir", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphEditorNode::cbSub, this, std::placeholders::_1));
  
  current_pose_graphs_pub_ = this->create_publisher<std_msgs::msg::String>("current_pose_graphs", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  
  std::thread{std::bind(&PoseGraphEditorNode::spinPoseGraphMerge, this, std::placeholders::_1), pgme_}.detach();
}

void PoseGraphEditorNode::spinPoseGraphMerge(std::shared_ptr<PoseGraphMergeEditor> p_ptr){
  
  auto executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(p_ptr);
  executor_->spin();
}

void PoseGraphEditorNode::spinPoseGraph(std::shared_ptr<PoseGraphEditor> p_ptr){
  
  auto executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(p_ptr);
  executor_->spin();
}

void PoseGraphEditorNode::cbSub(const std_msgs::msg::String::SharedPtr msg){
  
  std::string sub_str;
  // ss is an object of stringstream that references the S string.  
  std::stringstream ss(msg->data); 

  // Use while loop to check the getline() function condition.  
  while (std::getline(ss, sub_str, '/')) 
      // `str` is used to store the token string while ' ' whitespace is used as the delimiter.
      continue;;
  
  RCLCPP_DEBUG(this->get_logger(), "final sub str is: %s", sub_str.c_str());

  if( !pose_graphs_.insert(sub_str).second )
  { 
    RCLCPP_WARN(this->get_logger(), "Folder exists, ignore this pose graph: %s", msg->data.c_str());
    return;
  }
  else{
    RCLCPP_INFO(this->get_logger(), "Load pose graph dir: %s, generate pose graph number: %d", msg->data.c_str(), pose_graph_num_);
    std::string pose_graph_node_name = "pg_" + std::to_string(pose_graph_num_);
    auto a_pose_graph = std::make_shared<PoseGraphEditor>(pose_graph_node_name, msg->data);
    all_pose_graphs_[pose_graph_num_] = sub_str;
    std::thread{std::bind(&PoseGraphEditorNode::spinPoseGraph, this, std::placeholders::_1), a_pose_graph}.detach();

    //@ pub current pose graph architecture to panel
    std::string final_str = "";
    for(auto it=all_pose_graphs_.begin(); it!=all_pose_graphs_.end(); it++){
      std::string tmp_str = "";
      tmp_str += "pg_" + std::to_string((*it).first) + " ---> " + (*it).second + ";";
      final_str+=tmp_str;
    }
    std_msgs::msg::String to_pub;
    to_pub.data = final_str;
    current_pose_graphs_pub_->publish(to_pub);
    pose_graph_num_++;
    pgme_->pge_vector_.push_back(a_pose_graph);
  }
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto PGEN = std::make_shared<PoseGraphEditorNode>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(PGEN);
  executor.spin();

  return 0;
}


