#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <mrs_lib/service_client_handler.h>
#include <vector>
#include <std_srvs/srv/trigger.hpp>

namespace mrs_uav_trajectory_loader {

class TrajectoryLoaderNode : public rclcpp::Node {
public:
  explicit TrajectoryLoaderNode(const rclcpp::NodeOptions & opts);

private:
  void onInit();

  rclcpp::TimerBase::SharedPtr init_timer_;

  std::string mode_;
  std::string file_path_;          // path to .txt trajectory file
  std::string frame_id_;
  std::string service_name_;

  bool use_heading_ = true;
  bool fly_now_     = false;
  bool loop_        = false;     
  double dt_        = 0.2;
  std::vector<double> offset_ = {0.0, 0.0, 0.0, 0.0};

  std::string service_load_;
  std::string service_goto_;
  std::string service_track_;
  std::string service_stop_;

  rclcpp::Client<mrs_msgs::srv::TrajectoryReferenceSrv>::SharedPtr client_traj_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_trig_;
};

}  // namespace mrs_uav_trajectory_loader

// Register component
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_trajectory_loader::TrajectoryLoaderNode)