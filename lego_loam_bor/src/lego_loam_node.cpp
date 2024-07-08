#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"
#include "transformFusion.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjection>("lego_loam_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("lego_loam_fa", projection_out_channel, association_out_channel);
  auto MO = std::make_shared<MapOptimization>("lego_loam_mo", association_out_channel);
  auto TF = std::make_shared<TransformFusion>("lego_loam_tf");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(IP);
  executor.add_node(FA);
  executor.add_node(MO);
  executor.add_node(TF);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}


