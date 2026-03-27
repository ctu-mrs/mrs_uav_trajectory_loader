#include <fstream>
#include <memory>
#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <optional>
#include <string_view>
#include <vector>

namespace mrs_uav_trajectory_loader {

class TrajectoryLoader : public mrs_lib::Node {
public:
  TrajectoryLoader(rclcpp::NodeOptions options);

private:
  std::atomic<bool> m_is_init = false;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::CallbackGroup::SharedPtr m_cbgrp;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv>
      m_service_client_load_traj;

  std::shared_ptr<mrs_lib::ROSTimer> m_timer_loader;
  void cb_timer_loader();

  mrs_msgs::msg::TrajectoryReference m_traj_ref;

  std::optional<std::vector<double>> parseLine(const std::string &line);
  std::vector<mrs_msgs::msg::Reference>
  load_from_file(std::ifstream &f_obj, const std::vector<double> &offset);

  // void run(const std::string_view &type);
};

TrajectoryLoader::TrajectoryLoader(rclcpp::NodeOptions options)
    : mrs_lib::Node("trajectory_loader", options) {

  m_node = this_node_ptr();
  m_clock = m_node->get_clock();

  RCLCPP_INFO(m_node->get_logger(), "Initializing");

  m_cbgrp = m_node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::ParamLoader param_loader(m_node, m_node->get_name());
  param_loader.addYamlFileFromParam("default_config");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    RCLCPP_INFO(m_node->get_logger(), "Loading custom config %s",
                custom_config_path.c_str());

    param_loader.addYamlFile(custom_config_path);
  }

  std::string uav_name = param_loader.loadParam2<std::string>("uav_name");
  std::string file_path = param_loader.loadParam2<std::string>("traj_file");
  std::string mode = param_loader.loadParam2<std::string>("trajectory/mode");

  m_traj_ref.header.frame_id =
      uav_name + "/" +
      param_loader.loadParam2("trajectory/frame_id", std::string(""));

  param_loader.loadParam("trajectory/use_heading", m_traj_ref.use_heading,
                         false);
  param_loader.loadParam("trajectory/fly_now", m_traj_ref.fly_now, false);
  param_loader.loadParam("trajectory/dt", m_traj_ref.dt, 0.2);
  param_loader.loadParam("trajectory/loop", m_traj_ref.loop, false);

  std::vector<double> offset = param_loader.loadParam2<std::vector<double>>(
      "trajectory/offset", std::vector<double>{0.0, 0.0, 0.0, 0.0});

  // param_loader.loadParam("service.load_name",
  // service_load_,
  //                        std::string("/") + get_name() +
  //                            "/control_manager/trajectory_reference");
  // param_loader.loadParam("service.goto_name",
  // service_goto_,
  //                        std::string("/") + get_name() +
  //                            "/control_manager/goto_trajectory_start");
  // param_loader.loadParam("service.track_name",
  // service_track_,
  //                        std::string("/") + get_name() +
  //                            "/control_manager/start_trajectory_tracking");
  // param_loader.loadParam("service.stop_name",
  // service_stop_,
  //                        std::string("/") + get_name() +
  //                            "/control_manager/stop_trajectory_tracking");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_FATAL(m_node->get_logger(), "Missing required parameters");
    rclcpp::shutdown();
    return;
  }

  m_service_client_load_traj =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv>(
          m_node, "~/load_traj");

  mrs_lib::TimerHandlerOptions timer_opts;

  timer_opts.node = m_node;
  timer_opts.autostart = false;
  timer_opts.callback_group = m_cbgrp;

  m_timer_loader = std::make_shared<mrs_lib::ROSTimer>(
      timer_opts, rclcpp::Rate(1, m_clock),
      std::bind(&TrajectoryLoader::cb_timer_loader, this));

  if (offset.size() != 4) {
    RCLCPP_FATAL(m_node->get_logger(), "'trajectory/offset' must have size 4");
    rclcpp::shutdown();
    return;
  }

  std::ifstream fin(file_path);
  if (!fin) {
    RCLCPP_FATAL(m_node->get_logger(), "Cannot open trajectory file: %s",
                 file_path.c_str());
    rclcpp::shutdown();
    return;
  }

  m_traj_ref.header.stamp = rclcpp::Time(0);
  m_traj_ref.points = load_from_file(fin, offset);

  if (mode == "load") {
    m_timer_loader->start();
  } else {
  }

  RCLCPP_INFO(m_node->get_logger(), "Node initialized");
}

void TrajectoryLoader::cb_timer_loader() {

  auto sc_name = m_service_client_load_traj.getServiceName();

  if (!m_service_client_load_traj.waitForService(std::chrono::seconds(60))) {
    RCLCPP_FATAL(m_node->get_logger(), "Service %s not found", sc_name.c_str());
    rclcpp::shutdown();
    return;
  }

  auto req = std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Request>();
  req->trajectory = m_traj_ref;

  auto response = m_service_client_load_traj.callSync(req);

  if (!response) {

    RCLCPP_WARN(m_node->get_logger(), "Service %s did not respond",
                sc_name.c_str());
  } else {

    if (response.value()->success) {

      RCLCPP_INFO(m_node->get_logger(), "Service %s returned success",
                  sc_name.c_str());
    } else {

      RCLCPP_INFO(m_node->get_logger(),
                  "Service %s returned failure with msg: %s", sc_name.c_str(),
                  response.value()->message.c_str());
    }
  }
  RCLCPP_INFO(m_node->get_logger(),
              "Loaded trajectory reference using service %s", sc_name.c_str());
  m_timer_loader->stop();
}

std::optional<std::vector<double>>
TrajectoryLoader::parseLine(const std::string &line) {
  std::istringstream ss(line);
  std::string token;
  std::vector<double> values;

  while (std::getline(ss, token, ',')) {
    token.erase(0, token.find_first_not_of(" \t\r\n"));
    token.erase(token.find_last_not_of(" \t\r\n") + 1);
    if (token.empty())
      continue;

    try {
      values.push_back(std::stod(token));
    } catch (...) {
      return std::nullopt;
    }
  }

  if (values.size() < 4)
    return std::nullopt;

  return std::make_optional(values);
}

std::vector<mrs_msgs::msg::Reference>
TrajectoryLoader::load_from_file(std::ifstream &f_obj,
                                 const std::vector<double> &offset) {

  std::vector<mrs_msgs::msg::Reference> ref_points;

  std::string line;
  while (std::getline(f_obj, line)) {
    // ignore empty lines and comments
    if (line.empty() || line.at(0) == '#')
      continue;

    // safe parsing of line
    auto ret = parseLine(line);
    if (ret) {
      auto &values = ret.value();
      mrs_msgs::msg::Reference ref;

      ref.position.x = values.at(0) + offset.at(0);
      ref.position.y = values.at(1) + offset.at(1);
      ref.position.z = values.at(2) + offset.at(2);
      ref.heading = values.at(3) + offset.at(3);

      ref_points.push_back(ref);
    } else {
      RCLCPP_WARN(m_node->get_logger(), "Skipping bad line: %s", line.c_str());
    }
  }

  return ref_points;
}

} // namespace
  // mrs_uav_trajectory_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_trajectory_loader::TrajectoryLoader)
