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
#include <mrs_lib/vector_converter.h>

//}

using boost::lexical_cast;
using std::string;
namespace po = boost::program_options;

/* class TrajectoryLoader //{ */

class TrajectoryLoader {
private:

  struct traj_cfg_t
  {
    std::string uav_name;
    std::vector<double> offset;
    bool use_heading;
    bool fly_now;
    double dt;
    ros::Duration delay;
    bool loop;
    std::string frame_id;
    std::string service_name;
  } default_traj_cfg;

private:
  ros::NodeHandle _nh_;

  /* trajectory parameters */
  std::string _current_working_directory_;
  bool        _use_heading_;  // whether to fly trajectory with desired heading
  bool        _fly_now_;      // whether to start tracking the trajectory immediately after loading
  bool        _loop_;         // whether to fly trajectory in loop (infinitely times)
  double      _dt_;

  /* config parameters */
  ros::Duration            _timeout_;
  std::string              _uav_name_;
  std::string              _load_service_name_;
  std::vector<std::string> _uav_name_list_;

  std::mutex result_info_mtx_;
  std::vector<int> result_info_list_;
  std::vector<mrs_msgs::TrajectoryReference> trajectories_list_;
  std::vector<ros::ServiceClient>            service_client_list_;
  
  /* timers for asynchronous calling */
  std::vector<ros::Timer> timer_list_;

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

    ROS_INFO("- Loading trajectory from %s", filename.c_str());

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


  /* serviceNameFromUav() method //{ */
  std::string serviceNameFromUav(const std::string& uav_name)
  {
    return "/" + uav_name + "/" + _load_service_name_;
  }
  //}

  /* publishTrajectory() method //{ */

  // Method for publishing trajectory msg on specific service topic
  void publishTrajectory([[maybe_unused]] const ros::TimerEvent &event, const size_t index, const mrs_msgs::TrajectoryReference &trajectory, const traj_cfg_t& cfg)
  {
    const std::string service_name = serviceNameFromUav(cfg.uav_name);
    ROS_INFO("%s: Calling service: %s", cfg.uav_name.c_str(), service_name.c_str());

    mrs_msgs::TrajectoryReferenceSrv srv;
    srv.request.trajectory = trajectory;

    // calling service
    auto sc = _nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>(service_name);
    if (sc.call(srv))
    {
      // when the calling was successful and the client received response
      if (srv.response.success)
      {
        if (srv.response.modified)
          ROS_WARN("%s: %s", cfg.uav_name.c_str(), srv.response.message.c_str());
        else
          ROS_INFO("%s: %s", cfg.uav_name.c_str(), srv.response.message.c_str());
      } else
      {
        ROS_ERROR("%s: %s", cfg.uav_name.c_str(), srv.response.message.c_str());
      }
    }
    else
    {
      // service calling was unsuccessful because other side doesn't exist
      ROS_ERROR("%s: Failed to call service", cfg.uav_name.c_str());
    }
    
    // Finally, indicate that we are done
    std::scoped_lock lck(result_info_mtx_);
    result_info_list_.at(index) = true;
  }

  //}

  void callServiceTrigger(const ros::TimerEvent &event, const int index);

  /* timeoutFunction() method //{ */

  // Method that checks results from service threads and also checks if calling doesn't take too much time
  void timeoutFunction([[maybe_unused]] const ros::TimerEvent &event)
  {
  }

  //}

  /* serviceTimeoutCheck() method //{ */
  void serviceTimeoutCheck(const ros::Duration& timeout)
  {
    ROS_INFO("Main thread: Starting a %.1fs timeout.", timeout.toSec());
    auto timer = _nh_.createTimer(timeout, boost::bind(
            &TrajectoryLoader::timeoutFunction, this
            ), true, true);  // the last boolean argument makes the timer run only once);
  
    const ros::Duration timestep(0.1);
    ros::Duration current_timeout(0);
  
    ros::AsyncSpinner spinner(timer_list_.size());
    spinner.start();
  
    while (current_timeout < timeout) {
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

public:
  TrajectoryLoader();

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
    
    const auto current_working_directory = param_loader.loadParam2<bool>("current_working_directory" );
    default_traj_cfg.uav_name = _uav_name_;
    default_traj_cfg.dt = 0.2;
    const auto cfg_common = load_traj_cfg(param_loader, "trajectory", default_traj_cfg);
    
    //}

    /* Load all the trajectories and their optional configurations //{ */
    
    bool all_trajs_loaded = true;
    std::vector<std::pair<mrs_msgs::TrajectoryReference, traj_cfg_t>> loaded_trajs;
    for (auto& uav_name : _uav_name_list_)
    {
      auto cfg = load_traj_cfg(param_loader, "trajectory/uavs/"+uav_name, cfg_common);
    
      const auto traj_fname = param_loader.loadParam2<std::string>("trajectory/uavs/"+uav_name+"/filename");
      const auto msg_opt = loadTrajectoryFromFile(traj_fname, cfg);
      if (!msg_opt)
      {
        all_trajs_loaded = false;
        break;
      }
    
      loaded_trajs.push_back({msg_opt.value(), cfg});
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
    for (size_t it = 0; it < loaded_trajs.size(); it++)
    {
      const auto& [msg, cfg] = loaded_trajs.at(it);
      result_info_list_.push_back(false);
      timer_list_.push_back(
          _nh_.createTimer(cfg.delay, boost::bind(
              &TrajectoryLoader::publishTrajectory, this,
              _1, _2,  _3,
              it, msg, cfg
              ), true, true));  // the last boolean argument makes the timer run only once);
      ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", cfg.uav_name.c_str(), cfg.delay.toSec(), serviceNameFromUav(cfg.uav_name).c_str());
    
      if (cfg.delay > max_delay)
        max_delay = cfg.delay;
    }
    
    //}
  
    // Start the timeout function
    const ros::Duration timeout = max_delay + _timeout_;
    serviceTimeoutCheck(timeout);
  }

  //}

  void gotoStart()
  {
    string topic;
    ros::ServiceClient sc;
    for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
      topic = "/" + _uav_name_list_[i] + "/" + _service_topic_;
      sc    = _nh_.serviceClient<std_srvs::Trigger>(topic.c_str());
      service_client_list_.push_back(sc);
      result_info_list_.push_back(false);
    }

    /* call services using individual threads */
    for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
      if (_delay_list_[i] > 1e-3) {
        ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", _uav_name_list_[i].c_str(), _delay_list_[i],
                 service_client_list_[i].getService().c_str());
      }
      timer_list_.push_back(_nh_.createTimer(ros::Duration(_delay_list_[i]), boost::bind(&TrajectoryLoader::callServiceTrigger, this, _1, i), true,
                                              true));  // the last boolean argument makes the timer run only once);
    }

    // Start the timeout function
    const ros::Duration timeout = max_delay + _timeout_;
    serviceTimeoutCheck(timeout);
  };

  void startTracking() {};
};

//}

/* TrajectoryLoader::TrajectoryLoader() //{ */

TrajectoryLoader::TrajectoryLoader() {
  _nh_ = ros::NodeHandle("~");

  ROS_INFO("Loading general parameters:");
  mrs_lib::ParamLoader param_loader(_nh_);

  param_loader.loadParam("uav_name", _uav_name_, std::string());
  param_loader.loadParam("service/timeout", _timeout_, ros::Duration(5.0));
  param_loader.loadParam("service/load_name", _load_service_name_);
  param_loader.loadParam("service/goto_name", _load_service_name_);

  const auto trajs_cfgs = param_loader.loadParam2<XmlRpc::XmlRpcValue>("trajectory/uavs", {});
  for (auto& el : trajs_cfgs)
    _uav_name_list_.push_back(el.first); // name of the parameter, containing configuration of an uav's trajectory, is the uav's name

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("Could not load all non-optional parameters!");
    ros::shutdown();
    return;
  }

  // sanity checking
  if (_timeout_ < ros::Duration(0))
  {
    ROS_ERROR("The parameter 'service/timeout' has to be positive!");
    ros::shutdown();
    return;
  }
}

//}

/* /1* TrajectoryLoader::loadMultipleTrajectories() //{ *1/ */

/* // Main method for loading trajectories. This method creates independent thread for each uav. */
/* void TrajectoryLoader::loadMultipleTrajectories() */ 

/* //} */

/* /1* TrajectoryLoader::callMultipleServiceTriggers() //{ *1/ */

/* // Main method for calling Trigger service. This method creates independent thread for each calling. */
/* void TrajectoryLoader::callMultipleServiceTriggers() { */

/*   // | ------------------------ services ------------------------ | */
/*   string             topic; */
/*   ros::ServiceClient sc; */
/*   for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) { */
/*     topic = "/" + _uav_name_list_[i] + "/" + _service_topic_; */
/*     sc    = _nh_.serviceClient<std_srvs::Trigger>(topic.c_str()); */
/*     service_client_list_.push_back(sc); */
/*     result_info_list_.push_back(false); */
/*   } */

/*   /1* call services using individual threads *1/ */
/*   for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) { */
/*     if (_delay_list_[i] > 1e-3) { */
/*       ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", _uav_name_list_[i].c_str(), _delay_list_[i], */
/*                service_client_list_[i].getService().c_str()); */
/*     } */
/*     timer_list_.push_back(_nh_.createTimer(ros::Duration(_delay_list_[i]), boost::bind(&TrajectoryLoader::callServiceTrigger, this, _1, i), true, */
/*                                             true));  // the last boolean argument makes the timer run only once); */
/*   } */

/*   /1* timeout for threads *1/ */
/*   timeoutFunction(); */
/* } */

/* //} */

/* /1* TrajectoryLoader::callServiceTrigger() //{ *1/ */

/* // Method for calling service msg Trigger on specific topic */
/* void TrajectoryLoader::callServiceTrigger([[maybe_unused]] const ros::TimerEvent &event, const int index) { */
/*   ROS_INFO("%s: Calling service: %s", _uav_name_list_[index].c_str(), service_client_list_[index].getService().c_str()); */

/*   std_srvs::Trigger srv; */
/*   // calling service */
/*   if (service_client_list_[index].call(srv)) { */
/*     // when the calling was successful and the client received response */
/*     if (srv.response.success) { */
/*       ROS_INFO("%s: %s", _uav_name_list_[index].c_str(), srv.response.message.c_str()); */
/*     } else { */
/*       ROS_ERROR("%s: %s", _uav_name_list_[index].c_str(), srv.response.message.c_str()); */
/*     } */
/*     // service calling was unsuccessful because other side doesn't exist */
/*   } else { */
/*     ROS_ERROR("%s: Failed to call service", _uav_name_list_[index].c_str()); */
/*   } */
/*   result_info_list_.at(index) = true; */
/* } */

/* //} */

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

  if (mode == "load")
    trajectory_loader.loadTrajectories();
  else if (mode == "flyto")
    trajectory_loader.flyToStart();
  else if (mode == "track")
    trajectory_loader.startTracking();
  else
  {
    ROS_ERROR(" Unknown mode: '%s'! Valid options are 'load' or 'start'.", mode.c_str());
    return 1;
  }
  ros::shutdown();
};

//}
