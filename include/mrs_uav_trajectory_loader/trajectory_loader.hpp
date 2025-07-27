#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <mrs_lib/service_client_handler.h>

namespace mrs_uav_trajectory_loader {

class TrajectoryLoaderNode : public rclcpp::Node {
public:
  explicit TrajectoryLoaderNode(const rclcpp::NodeOptions & opts);

private:
  void onInit();

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::string file_path_;          // path to .txt trajectory file
  std::string frame_id_;
  std::string service_name_;

  rclcpp::Client<mrs_msgs::srv::TrajectoryReferenceSrv>::SharedPtr client_;
};

}  // namespace mrs_uav_trajectory_loader

// Register component
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_trajectory_loader::TrajectoryLoaderNode)