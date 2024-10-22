/* includes //{ */

#include <ros/ros.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <std_srvs/Trigger.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <mutex>
#include <boost/chrono/chrono_io.hpp>

#include <mrs_lib/param_loader.h>

//}

using boost::lexical_cast;
using std::string;
namespace po = boost::program_options;

/* class TrajectoryLoader //{ */

class TrajectoryLoader {
private:

  struct traj_cfg_t
  {
    std::string uav_name = "";
    std::vector<double> offset = {0,0,0,0};
    bool use_heading = true;
    bool fly_now = false;
    double dt = 0.2;
    ros::Duration delay = ros::Duration{0};
    bool loop = false;
    std::string frame_id = "";
    std::string service_name = "";
  } default_traj_cfg;

private:
  ros::NodeHandle _nh_;

  /* Loaded parameters */
  ros::Duration _timeout_;
  std::string _uav_name_;
  std::string _load_service_name_;
  std::string _goto_service_name_;
  std::string _track_service_name_;
  std::string _stop_service_name_;
  std::vector<std::string> _uav_name_list_;
  // Indicates whether the UAV name list is from the _uav_name_ (which is usually set dynamically using an environment variable)
  // or loaded from the "trajectory/uavs" parameter list. If it's the first case, then the trajectory filename is loaded from
  // parameter "trajectory/filename", else it's loaded from "trajectory/uavs/X/filename" for each UAV name X.
  bool _dynamic_uav_name_;

  /* working variables */
  /* timers for delayed service calling */
  std::vector<ros::Timer> timer_list_;
  /* results of service calls will be stored here */
  std::mutex result_info_mtx_;
  std::vector<int> result_info_list_;

  /* loadTrajectoryFromFile() method //{ */

  // method for loading trajectories from file into mrs_msgs/TrajectoryReference msg
  std::optional<mrs_msgs::TrajectoryReference> loadTrajectoryFromFile(const string &filename, const traj_cfg_t& cfg)
  {
    std::ifstream file_in(filename.c_str(), std::ifstream::in);

    if (!file_in)
    {
      ROS_ERROR("- Cannot open %s", filename.c_str());
      return std::nullopt;
    }

    std::string line;
    mrs_msgs::TrajectoryReference new_traj;
    mrs_msgs::Reference point;
    std::vector<string> parts;

    /* for each line in file */
    while (getline(file_in, line))
    {
      // replace comma with space
      boost::replace_all(line, ",", " ");
      // if now line contains two space replace them with one space
      boost::replace_all(line, "  ", " ");
      // remove ^M is Windows carriage return
      boost::replace_all(line, "\r", "");
      // split string using space delimiter
      boost::split(parts, line, boost::is_any_of(" "));

      if (parts.size() != 4)
      {
        ROS_ERROR("- Incorrect format of the trajectory on line %d", int(new_traj.points.size() + 1));
        return std::nullopt;
      }

      try
      {
        point.position.x = lexical_cast<double>(parts[0]);
        point.position.y = lexical_cast<double>(parts[1]);
        point.position.z = lexical_cast<double>(parts[2]);
        point.heading    = lexical_cast<double>(parts[3]);
      }
      catch (...)
      {
        ROS_ERROR("- Some error occured during reading line %d", int(new_traj.points.size() + 1));
        return std::nullopt;
      }

      point.position.x += cfg.offset.at(0);
      point.position.y += cfg.offset.at(1);
      point.position.z += cfg.offset.at(2);
      point.heading += cfg.offset.at(3);

      new_traj.points.push_back(point);
    }
    file_in.close();

    new_traj.header.stamp = ros::Time(0);
    new_traj.header.frame_id = cfg.frame_id;
    new_traj.use_heading  = cfg.use_heading;
    new_traj.fly_now      = cfg.fly_now;
    new_traj.dt           = cfg.dt;
    new_traj.loop         = cfg.loop;

    return new_traj;
  }

  //}

  /* concatServiceName() method //{ */
  std::string concatServiceName(const std::string& uav_name, const std::string& service_name)
  {
    return "/" + uav_name + "/" + service_name;
  }
  //}

  /* serviceTimeoutCheck() method //{ */
  void serviceTimeoutCheck(const ros::Duration& timeout)
  {
    ROS_INFO("Main thread: Starting a %.1fs timeout.", timeout.toSec());
  
    const ros::Duration timestep(0.1);
    ros::Duration current_timeout(0);
  
    ros::AsyncSpinner spinner(timer_list_.size());
    spinner.start();
  
    while (current_timeout < timeout)
    {
      timestep.sleep();
      current_timeout += timestep;
  
      bool total_result = true;
      std::scoped_lock lck(result_info_mtx_);
      for (auto& el : result_info_list_)
        total_result = total_result && el;
  
      if (total_result)
      {
        ROS_INFO("Main thread: All service threads finished. Ending.");
        return;
      }
    }
  
    ROS_ERROR("Main thread: Timeout reached --> terminating all remaining threads");
    for (size_t it = 0; it < timer_list_.size(); ++it)
    {
      if (!result_info_list_.at(it))
      {
        ROS_ERROR("%s: Deadlock during calling. Forcing the thread down.", _uav_name_list_.at(it).c_str());
        timer_list_.at(it).stop();
      }
    }
  };
  //}

  /* callService() method //{ */

  // Method for calling service msg Trigger on specific topic
  template<typename T>
  void callService([[maybe_unused]] const ros::TimerEvent &event, const std::string& uav_name, const std::string& service_name_part, T srv, const size_t index)
  {
    const std::string service_name = concatServiceName(uav_name, service_name_part);
    ROS_INFO("%s: Calling service: \"%s\"", uav_name.c_str(), service_name.c_str());

    // calling service
    auto sc = _nh_.serviceClient<T>(service_name);
    if (sc.call(srv))
    {
      // when the calling was successful and the client received response
      if (srv.response.success)
      {
        ROS_INFO("%s: %s", uav_name.c_str(), srv.response.message.c_str());
      } else
      {
        ROS_ERROR("%s: %s", uav_name.c_str(), srv.response.message.c_str());
      }
    }
    else
    {
      // service calling was unsuccessful because other side doesn't exist
      ROS_ERROR("%s: Failed to call service", uav_name.c_str());
    }
    
    // Finally, indicate that we are done
    std::scoped_lock lck(result_info_mtx_);
    result_info_list_.at(index) = true;
  }

  //}

  /* callServiceTriggersDelayed() method //{ */
  void callServiceTriggersDelayed(const std::string& service_name_part)
  {
    mrs_lib::ParamLoader param_loader(_nh_);
  
    // load the trajectory delays
    const auto default_delay = param_loader.loadParam2<ros::Duration>("trajectory/delay", ros::Duration(0));
    std::vector<ros::Duration> delay_list;
    for (auto& uav_name : _uav_name_list_)
    {
      const auto delay = param_loader.loadParam2<ros::Duration>("trajectory/uavs/"+uav_name+"/delay", default_delay);
      delay_list.push_back(delay);
    }
  
    // if some of the trajectories failed to load, it's a failure, end
    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("Failed to load some compulsory parameters. Ending.");
      return;
    }
  
    /* If all is fine so far, start the delayed loading of trajectories //{ */
  
    ros::Duration max_delay(0); // also use this opportunity to find the maximal delay
    for (size_t it = 0; it < _uav_name_list_.size(); it++)
    {
      const auto uav_name = _uav_name_list_.at(it);
      const auto delay = delay_list.at(it);
      const auto fn = boost::bind(
              &TrajectoryLoader::callService<std_srvs::Trigger>, this,
              _1, uav_name, service_name_part, std_srvs::Trigger{}, it
              );

      result_info_list_.push_back(false);
      timer_list_.push_back(
          _nh_.createTimer(delay, fn, true, true));  // the last boolean argument makes the timer run only once);
      ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", uav_name.c_str(), delay.toSec(), concatServiceName(uav_name, service_name_part).c_str());
  
      if (delay > max_delay)
        max_delay = delay;
    }
  
    //}
  
    // Start the timeout function
    const ros::Duration timeout = max_delay + _timeout_;
    serviceTimeoutCheck(timeout);
  };
  //}

public:
  /* init() method //{ */

  bool init(const std::string& mode)
  {
    _nh_ = ros::NodeHandle("~");

    ROS_INFO("Loading general parameters:");
    mrs_lib::ParamLoader param_loader(_nh_);

    param_loader.loadParam("uav_name", _uav_name_, std::string());
    param_loader.loadParam("service/timeout", _timeout_, ros::Duration(5.0));
    param_loader.loadParam("service/load_name", _load_service_name_);
    param_loader.loadParam("service/goto_name", _goto_service_name_);
    param_loader.loadParam("service/track_name", _track_service_name_);
    param_loader.loadParam("service/stop_name", _stop_service_name_);
    param_loader.loadParam("trajectory/dynamic_uav_name", _dynamic_uav_name_);

    const auto trajs_cfgs_exist = _nh_.hasParam("trajectory/uavs");
    std::string uav_names;
    ROS_INFO("%d %d",trajs_cfgs_exist,_dynamic_uav_name_);
    if (trajs_cfgs_exist && !_dynamic_uav_name_)
    {
      const auto trajs_cfgs = param_loader.loadParam2<XmlRpc::XmlRpcValue>("trajectory/uavs");
      for (auto& el : trajs_cfgs)
      {
        _uav_name_list_.push_back(el.first);
        uav_names += " '" + el.first + "'";
      }
    }
    else
    {
      _uav_name_list_.push_back(_uav_name_);
      uav_names += " only '" + _uav_name_ + "'";
      _dynamic_uav_name_ = true; // name of the parameter, containing configuration of an uav's trajectory, "trajectory/filename"
    }
    if (mode == "load")
    {
      if (_dynamic_uav_name_)
        uav_names += " (using UAV name \033[1;33magnostic\033[0m parameter \033[1;33m\"trajectory/filename\"\033[0m to load filename of the trajectory)";
      else
        uav_names += " (using UAV name \033[1;33mspecific\033[0m parameters \033[1;33m\"trajectory/uavs/<uav_name>/filename\"\033[0m to load filename of the trajectory)";
    }

    ROS_INFO("Loaded UAV names:%s", uav_names.c_str());

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("Could not load all non-optional parameters!");
      return false;
    }

    // sanity checking
    if (_timeout_ < ros::Duration(0))
    {
      ROS_ERROR("The parameter 'service/timeout' has to be positive!");
      return false;
    }

    return true;
  }

  //}

  /* void loadTrajectories() method //{ */
  
  traj_cfg_t load_traj_cfg(mrs_lib::ParamLoader& pl, const string& ns, const traj_cfg_t& defaults)
  {
    traj_cfg_t cfg;
    pl.loadParam(ns+"/offset", cfg.offset, defaults.offset);
    pl.loadParam(ns+"/use_heading", cfg.use_heading, defaults.use_heading);
    pl.loadParam(ns+"/fly_now", cfg.fly_now, defaults.fly_now);
    pl.loadParam(ns+"/dt", cfg.dt, defaults.dt);
    pl.loadParam(ns+"/delay", cfg.delay, defaults.delay);
    pl.loadParam(ns+"/loop", cfg.loop, defaults.loop);
    pl.loadParam(ns+"/frame_id", cfg.frame_id, defaults.frame_id);
    return cfg;
  }
  
  // Loads trajectories from yaml files and sends them to the specified service
  void loadTrajectories()
  {
    mrs_lib::ParamLoader param_loader(_nh_);
  
    /* First, load some common parameters //{ */
    
    ROS_INFO("- Loading common trajectory parameters");
    const auto base_path = param_loader.loadParam2<std::string>("trajectory/base_path");
    const auto cfg_common = load_traj_cfg(param_loader, "trajectory", default_traj_cfg);
    
    //}

    /* Load all the trajectories and their optional configurations //{ */
    // First, load filenames of the trajectories
    std::vector<std::string> traj_fnames;
    if (_dynamic_uav_name_)
    // if the UAV name is dynamic, load the "trajectory/filename" parameter (which is independent of UAV name)
    {
      traj_fnames.push_back(param_loader.loadParam2<std::string>("trajectory/filename"));
    }
    else
    // otherwise, load the names from the static "trajectory/uavs/uav_name/filename" parameters
    {
      for (auto& uav_name : _uav_name_list_)
        traj_fnames.push_back(param_loader.loadParam2<std::string>("trajectory/uavs/"+uav_name+"/filename"));
    }

    // Check that we've gotten everything we needed
    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("Failed to load some compulsory parameters. Ending.");
      return;
    }
    
    bool all_trajs_loaded = true;
    std::vector<mrs_msgs::TrajectoryReference> loaded_trajs;
    std::vector<ros::Duration> traj_delays;
    for (size_t it = 0; it < _uav_name_list_.size(); it++)
    {
      const auto& uav_name = _uav_name_list_.at(it);
      const auto& traj_fname = traj_fnames.at(it);
      const auto pathfname = base_path + "/" + traj_fname;
      ROS_INFO("- Loading trajectory for UAV \033[1;33m\"%s\"\033[0m from \033[1;33m\"%s\"\033[0m", uav_name.c_str(), pathfname.c_str());

      auto cfg = load_traj_cfg(param_loader, "trajectory/uavs/"+uav_name, cfg_common);
      cfg.uav_name = uav_name;
      const auto msg_opt = loadTrajectoryFromFile(pathfname, cfg);
      if (!msg_opt)
      {
        all_trajs_loaded = false;
        break;
      }
    
      loaded_trajs.push_back(msg_opt.value());
      traj_delays.push_back(cfg.delay);
    }
    
    //}
  
    // if some of the trajectories failed to load, it's a failure, end
    if (!all_trajs_loaded)
    {
      ROS_ERROR("Failed to load some trajectories. Ending.");
      return;
    }
    ROS_INFO("All trajectories have been found.");
  
    /* If all is fine so far, start the delayed loading of trajectories //{ */
    
    ros::Duration max_delay(0); // also use this opportunity to find the maximal delay
    for (size_t it = 0; it < _uav_name_list_.size(); it++)
    {
      const auto& uav_name = _uav_name_list_.at(it);
      const auto& delay = traj_delays.at(it);
      const auto& msg = loaded_trajs.at(it);

      mrs_msgs::TrajectoryReferenceSrv srv;
      srv.request.trajectory = msg;
      const auto fn = boost::bind(
              &TrajectoryLoader::callService<mrs_msgs::TrajectoryReferenceSrv>, this,
              _1, uav_name, _load_service_name_, srv, it
              );

      result_info_list_.push_back(false);
      timer_list_.push_back(
          _nh_.createTimer(delay, fn, true, true));  // the last boolean argument makes the timer run only once);
      ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", uav_name.c_str(), delay.toSec(), concatServiceName(uav_name, _load_service_name_).c_str());
    
      if (delay > max_delay)
        max_delay = delay;
    }
    
    //}
  
    // Start the timeout function
    const ros::Duration timeout = max_delay + _timeout_;
    serviceTimeoutCheck(timeout);
  }

  //}

  void gotoStart()
  {
    callServiceTriggersDelayed(_goto_service_name_);
  };

  void startTracking()
  {
    callServiceTriggersDelayed(_track_service_name_);
  };

  void stopTracking()
  {
    callServiceTriggersDelayed(_stop_service_name_);
  };
};

//}

/* MAIN FUNCTION //{ */
int main(int argc, char **argv) {

  // initialize ROS
  ros::init(argc, argv, "trajectory_loader");
  ros::NodeHandle nh("~");
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh);
  const auto mode = param_loader.loadParam2<std::string>("mode");
  if (!param_loader.loadedSuccessfully())
    ROS_ERROR("Please specify the mode (load/start).");

  ROS_INFO("Node initialized.");

  // create class TrajectoryLoader
  TrajectoryLoader trajectory_loader;

  if (!trajectory_loader.init(mode))
    return 1;

  if (mode == "load")
    trajectory_loader.loadTrajectories();
  else if (mode == "goto")
    trajectory_loader.gotoStart();
  else if (mode == "track")
    trajectory_loader.startTracking();
  else if (mode == "stop")
    trajectory_loader.stopTracking();
  else
  {
    ROS_ERROR("Unknown mode: '%s'! Valid options are 'load', 'goto', 'track' and 'stop'.", mode.c_str());
    return 1;
  }
  ros::shutdown();
};

//}
