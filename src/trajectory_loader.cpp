#include "mrs_uav_trajectory_loader/trajectory_loader.hpp"
#include <fstream>
#include <boost/algorithm/string.hpp>

namespace mrs_uav_trajectory_loader {

TrajectoryLoaderNode::TrajectoryLoaderNode(const rclcpp::NodeOptions & opts)
: rclcpp::Node("trajectory_loader", opts)
{
  // delayed init to safely load params and call service
  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    std::bind(&TrajectoryLoaderNode::onInit, this)
  );
}


void TrajectoryLoaderNode::onInit() {
  // shared ptr for helpers
  auto node_ptr = this->shared_from_this();
  mrs_lib::ParamLoader pl(node_ptr, "TrajectoryLoaderNode");

  // -------- parameters --------
  pl.loadParam("trajectory.file", file_path_);
  pl.loadParam("trajectory.frame_id", frame_id_, std::string(""));
  pl.loadParam("service.load_name", service_name_, std::string("trajectory_loader.load"));

  if (!pl.loadedSuccessfully()) {
    RCLCPP_FATAL(this->get_logger(), "Missing required parameters");
    rclcpp::shutdown();
    return;
  }

  // -------- read file --------
  std::ifstream fin(file_path_);
  if (!fin) {
    RCLCPP_FATAL(get_logger(), "Cannot open trajectory file: %s", file_path_.c_str());
    rclcpp::shutdown();
    return;
  }

  mrs_msgs::srv::TrajectoryReferenceSrv::Request::SharedPtr req =
    std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Request>();
  auto & traj = req->trajectory;
  traj.header.stamp = rclcpp::Time(0);
  traj.header.frame_id = frame_id_;
  traj.use_heading = true;
  traj.fly_now     = false;
  traj.dt          = 0.0;
  traj.loop        = false;

  std::string line;
  std::vector<std::string> parts;
  while (std::getline(fin, line)) {
    boost::replace_all(line, ",", " ");
    boost::split(parts, line, boost::is_any_of(" 	"), boost::token_compress_on);
    if (parts.size() < 4) {
      RCLCPP_WARN(get_logger(), "Skipping malformed line: '%s'", line.c_str());
      continue;
    }
    mrs_msgs::msg::Reference ref;
    try {
      ref.position.x = std::stod(parts[0]);
      ref.position.y = std::stod(parts[1]);
      ref.position.z = std::stod(parts[2]);
      ref.heading    = std::stod(parts[3]);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Conversion error in line: '%s'", line.c_str());
      continue;
    }
    traj.points.push_back(ref);
  }

  // -------- service client --------
  client_ = this->create_client<mrs_msgs::srv::TrajectoryReferenceSrv>(service_name_);
  if (!client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "Service %s not available", service_name_.c_str());
  } else {
    auto future = client_->async_send_request(req);
    // Handle response asynchronously
    std::thread([future = std::move(future), this]() mutable {
      try {
        auto resp = future.get();
        if (resp->success)
          RCLCPP_INFO(this->get_logger(), "Trajectory accepted: %s", resp->message.c_str());
        else
          RCLCPP_ERROR(this->get_logger(), "Trajectory rejected: %s", resp->message.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    }).detach();
  }

  init_timer_->cancel();
}

}  // namespace mrs_uav_trajectory_loader