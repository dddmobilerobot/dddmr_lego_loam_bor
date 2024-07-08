#include "imageProjection.h"
#include "featureAssociation.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjection>("mcl_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("mcl_fa", projection_out_channel, association_out_channel);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(IP);
  executor.add_node(FA);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}


