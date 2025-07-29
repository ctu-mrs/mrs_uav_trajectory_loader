#include "mrs_uav_trajectory_loader/trajectory_loader.hpp"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mrs_uav_trajectory_loader {

TrajectoryLoaderNode::TrajectoryLoaderNode(const rclcpp::NodeOptions & opts)
: rclcpp::Node("trajectory_loader", opts)
{
  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    std::bind(&TrajectoryLoaderNode::onInit, this)
  );
}


void TrajectoryLoaderNode::onInit() {
  auto node_ptr = this->shared_from_this();
  mrs_lib::ParamLoader pl(node_ptr, "TrajectoryLoaderNode");

  // -------- parameters --------
  pl.loadParam("trajectory.mode", mode_);

  pl.loadParam("trajectory.file", file_path_);
  pl.loadParam("trajectory.frame_id", frame_id_, std::string(""));
  // pl.loadParam("service.load_name", service_name_, std::string("trajectory_loader.load"));

  pl.loadParam("trajectory.use_heading", use_heading_, true);
  pl.loadParam("trajectory.fly_now",     fly_now_,     false);
  pl.loadParam("trajectory.dt",          dt_,          0.2);
  pl.loadParam("trajectory.loop",        loop_,        false);
  pl.loadParam("trajectory.offset", offset_, std::vector<double>{0.0, 0.0, 0.0, 0.0});
  
  pl.loadParam("service.load_name", service_load_, std::string("/") + get_name() + "/control_manager/trajectory_reference");
  pl.loadParam("service.goto_name", service_goto_, std::string("/") + get_name() + "/control_manager/goto_trajectory_start");
  pl.loadParam("service.track_name", service_track_, std::string("/") + get_name() + "/control_manager/start_trajectory_tracking");
  pl.loadParam("service.stop_name", service_stop_, std::string("/") + get_name() + "/control_manager/stop_trajectory_tracking");

  RCLCPP_INFO(this->get_logger(), "Node name is: %s", this->get_name());

  if (offset_.size() != 4) {
    RCLCPP_FATAL(get_logger(), "'trajectory.offset' must have exactly 4 numbers");
    return;
  }

  if (!pl.loadedSuccessfully()) {
    RCLCPP_FATAL(this->get_logger(), "Missing required parameters");
    rclcpp::shutdown();
    return;
  }

  if (mode_ == "load") {
    // -------- read file --------
    std::ifstream fin(file_path_);
    if (!fin) {
      RCLCPP_FATAL(get_logger(), "Cannot open trajectory file: %s", file_path_.c_str());
      return;
    }
    
    // -------- prepare request --------
    auto req = std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Request>();
    auto & traj = req->trajectory;
    traj.header.stamp    = rclcpp::Time(0);
    traj.header.frame_id = frame_id_;
    traj.use_heading     = use_heading_;
    traj.fly_now         = fly_now_;
    traj.dt              = dt_;
    traj.loop            = loop_;

    std::string line;
    std::vector<std::string> parts;
    while (std::getline(fin, line)) {
      boost::replace_all(line, ",", " ");
      boost::split(parts, line, boost::is_any_of(" 	"), boost::token_compress_on);
      if (parts.size() < 4) continue;
      mrs_msgs::msg::Reference ref;
      ref.position.x = std::stod(parts[0]) + offset_[0];
      ref.position.y = std::stod(parts[1]) + offset_[1];
      ref.position.z = std::stod(parts[2]) + offset_[2];
      ref.heading    = std::stod(parts[3]) + offset_[3];
      traj.points.push_back(ref);
    }

    // -------- call load service --------
    client_traj_ = this->create_client<mrs_msgs::srv::TrajectoryReferenceSrv>(service_load_);
    if (!client_traj_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", service_load_.c_str());
    } else {
      auto fut = client_traj_->async_send_request(req);
      std::thread([fut = std::move(fut), this]() mutable {
        try {
          auto resp = fut.get();
          if (resp->success)
            RCLCPP_INFO(get_logger(), "Trajectory loaded");
          else
            RCLCPP_ERROR(get_logger(), "Load failed: %s", resp->message.c_str());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Exception in load service call");
        }
      }).detach();
    }

  } else if (mode_ == "goto" || mode_ == "track" || mode_ == "stop") {
    
    // -------- Choose trigger service --------
    std::string srv = (mode_ == "goto"  ? service_goto_
                         : mode_ == "track" ? service_track_
                                            : service_stop_);
    client_trig_ = this->create_client<std_srvs::srv::Trigger>(srv);
    if (!client_trig_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", srv.c_str());
    } else {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto fut = client_trig_->async_send_request(req);
      std::thread([fut = std::move(fut), this, srv]() mutable {
        try {
          auto resp = fut.get();
          if (resp->success)
            RCLCPP_INFO(get_logger(), "%s succeeded", srv.c_str());
          else
            RCLCPP_ERROR(get_logger(), "%s failed: %s", srv.c_str(), resp->message.c_str());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Exception in %s call", srv.c_str());
        }
      }).detach();
    }

  } else {
    RCLCPP_ERROR(get_logger(), "Unknown mode: %s", mode_.c_str());
  }

  init_timer_->cancel();
}

}  // namespace mrs_uav_trajectory_loader