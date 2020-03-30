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
#include <boost/chrono/chrono_io.hpp>

#include <mrs_lib/ParamLoader.h>

// only for UNIX
#include <pthread.h>
//}

using boost::lexical_cast;
using std::string;
namespace po = boost::program_options;

/* class TrajectoryLoader //{ */

class TrajectoryLoader {
private:
  ros::NodeHandle _nh_;

  /* trajectory parameters */
  std::string _current_working_directory_;
  bool        _use_heading_;  // whether to fly trajectory with desired heading
  bool        _fly_now_;      // whether to start tracking the trajectory immediately after loading
  bool        _loop_;         // whether to fly trajectory in loop (infinitely times)
  double      _dt_;

  /* config parameters */
  double                   _timeout_for_calling_services_;
  std::string              _uav_name_;
  std::vector<double>      _offset_list_;
  std::vector<double>      _delay_list_;
  std::string              _service_topic_;
  std::vector<std::string> _uav_name_list_;

  std::vector<bool>                          result_info_list_;
  std::vector<mrs_msgs::TrajectoryReference> trajectories_list_;
  std::vector<ros::ServiceClient>            service_client_list_;

  /* functions definition */
  bool loadTrajectoryFromFile(const std::string &filename, mrs_msgs::TrajectoryReference &trajectory);
  void publishTrajectory(const ros::TimerEvent &event, const int index);
  void callServiceTrigger(const ros::TimerEvent &event, const int index);
  void timeoutFunction();

  /* threads for asynchronous calling */
  std::vector<ros::Timer> thread_list_;

public:
  TrajectoryLoader();
  void loadMultipleTrajectories();
  void callMultipleServiceTriggers();
};

//}

/* TrajectoryLoader::TrajectoryLoader() //{ */

TrajectoryLoader::TrajectoryLoader() {
  _nh_ = ros::NodeHandle("~");

  ROS_INFO("Loading general parameters:");
  mrs_lib::ParamLoader param_loader(_nh_);

  param_loader.load_param("service_topic", _service_topic_);
  param_loader.load_param("uav_name", _uav_name_, std::string());
  param_loader.load_param("main/uav_name_list", _uav_name_list_);
  param_loader.load_param("main/delay", _delay_list_);
  param_loader.load_param("timeout_for_calling_services", _timeout_for_calling_services_, double(5));

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("Could not load all non-optional parameters!");
    ros::shutdown();
    return;
  }

  /* sanity checks //{ */

  if (_uav_name_list_.empty()) {
    if (_uav_name_.empty()) {
      ROS_ERROR("uav_name_list (target UAVs) is empty!");
      ros::shutdown();
      return;
    } else {
      ROS_WARN("uav_name_list (target UAVs) is empty -> setting the list to: [%s]", _uav_name_.c_str());
      _uav_name_list_.push_back(_uav_name_);
    }
  }

  if (_delay_list_.size() != _uav_name_list_.size()) {
    ROS_ERROR("delay_list (parameter 'main/delay') should have the same size as 'uav_name_list'!");
    ros::shutdown();
    return;
  }

  // check if the delays are positive values and if some delays are set
  double delay_sum = 0;
  for (unsigned long i = 0; i < _delay_list_.size(); i++) {
    delay_sum += _delay_list_.at(i);
    if (_delay_list_.at(i) < 0) {
      ROS_ERROR("delay has to be positive value!");
      ros::shutdown();
      return;
    }
  }

  char buff[100];
  if (delay_sum > 1e-5) {
    string delay_text = "Delays are set to: [ ";
    for (unsigned long i = 0; i < _delay_list_.size(); i++) {
      snprintf(buff, sizeof(buff), "%s: %.1f", _uav_name_list_[i].c_str(), _delay_list_[i]);
      delay_text += buff;
      if (i != _delay_list_.size() - 1) {
        delay_text += ", ";
      }
    }
    delay_text += "]";
    ROS_WARN("%s", delay_text.c_str());
  }

  if (_timeout_for_calling_services_ < 0) {
    ROS_ERROR("Parameter timeout_for_calling_services has to be positive value!");
    ros::shutdown();
    return;
  }

  //}
}

//}

/* TrajectoryLoader::loadTrajectoryFromFile //{ */

// method for loading trajectories from file into mrs_msgs/TrajectoryReference msg
bool TrajectoryLoader::loadTrajectoryFromFile(const string &filename, mrs_msgs::TrajectoryReference &trajectory) {

  std::ifstream file_in(filename.c_str(), std::ifstream::in);

  if (!file_in) {
    ROS_ERROR("- Cannot open %s", filename.c_str());
    ros::shutdown();
    return false;

  } else {

    ROS_INFO("- Loading trajectory from %s", filename.c_str());

    string                        line;
    mrs_msgs::TrajectoryReference new_traj;
    mrs_msgs::Reference           point;
    std::vector<string>           parts;

    /* for each line in file */
    while (getline(file_in, line)) {
      // replace comma with space
      boost::replace_all(line, ",", " ");
      // if now line contains two space replace them with one space
      boost::replace_all(line, "  ", " ");
      // remove ^M is Windows carriage return
      boost::replace_all(line, "\r", "");
      // split string using space delimiter
      boost::split(parts, line, boost::is_any_of(" "));

      if (parts.size() != 4) {
        ROS_ERROR("- Incorrect format of the trajectory on line %d", int(new_traj.points.size() + 1));
        ros::shutdown();
        return false;
      }

      try {
        point.position.x = lexical_cast<double>(parts[0]);
        point.position.y = lexical_cast<double>(parts[1]);
        point.position.z = lexical_cast<double>(parts[2]);
        point.heading    = lexical_cast<double>(parts[3]);
      }
      catch (...) {
        ROS_ERROR("- Some error occured during reading line %d", int(new_traj.points.size() + 1));
        ros::shutdown();
        return false;
      }

      point.position.x += _offset_list_[0];
      point.position.y += _offset_list_[1];
      point.position.z += _offset_list_[2];
      point.heading += _offset_list_[3];

      new_traj.points.push_back(point);
    }
    file_in.close();

    new_traj.header.stamp = ros::Time(0);
    new_traj.use_heading  = _use_heading_;
    new_traj.fly_now      = _fly_now_;
    new_traj.dt           = _dt_;
    new_traj.loop         = _loop_;
    trajectory            = new_traj;
  }
  return true;
}

//}

/* TrajectoryLoader::loadMultipleTrajectories() //{ */

// Main method for loading trajectories. This method creates independent thread for each uav.
void TrajectoryLoader::loadMultipleTrajectories() {

  // | ------------------------- params ------------------------- |
  mrs_lib::ParamLoader param_loader(_nh_);

  param_loader.load_param("use_heading", _use_heading_, bool(false));
  param_loader.load_param("fly_now", _fly_now_, bool(false));
  param_loader.load_param("dt", _dt_);
  param_loader.load_param("loop", _loop_, bool(false));
  param_loader.load_param("main/offset", _offset_list_);
  param_loader.load_param("current_working_directory", _current_working_directory_);

  string text;
  string filename;
  string filename_array[_uav_name_list_.size()];
  if (_uav_name_list_.size() != 1 || (_uav_name_list_.size() == 1 && _uav_name_ != _uav_name_list_[0])) {
    for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
      text = _uav_name_list_[i] + "/filename";
      param_loader.load_param(text.c_str(), filename);
      filename_array[i] = _current_working_directory_ + filename;
    }
  } else {
    param_loader.load_param("filename", filename);
    filename_array[0] = _current_working_directory_ + filename;
  }

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("Could not load all non-optional parameters!");
    ros::shutdown();
    return;
  }

  /* sanity checks //{ */

  if (_offset_list_.size() != 4) {
    ROS_ERROR("Parameter offset should be set to [x,y,z,heading]!");
    ros::shutdown();
    return;
  }

  double offset_sum = _offset_list_[0] + _offset_list_[1] + _offset_list_[2] + _offset_list_[3];
  if (offset_sum > 1e-5) {
    ROS_WARN("Trajectories will be offsetted by [%.2f, %.2f, %.2f, %.2f]", _offset_list_[0], _offset_list_[1], _offset_list_[2], _offset_list_[3]);
  }

  /* load trajectories */
  bool trajectory_sucessfully_loaded = true;
  ROS_INFO("Loading trajectories from files:");
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    mrs_msgs::TrajectoryReference msg;
    trajectory_sucessfully_loaded &= loadTrajectoryFromFile(filename_array[i], msg);
    trajectories_list_.push_back(msg);
  }

  if (!trajectory_sucessfully_loaded) {
    return;
  }
  ROS_INFO("All trajectories have been found.\n");

  //}

  // | ------------------------ services ------------------------ |
  string             topic;
  ros::ServiceClient sc;
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    topic = "/" + _uav_name_list_[i] + "/" + _service_topic_;
    sc    = _nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>(topic.c_str());
    service_client_list_.push_back(sc);
    result_info_list_.push_back(false);
  }

  /* call services using individual threads */
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    if (_delay_list_[i] > 1e-3) {
      ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", _uav_name_list_[i].c_str(), _delay_list_[i],
               service_client_list_[i].getService().c_str());
    }
    thread_list_.push_back(_nh_.createTimer(ros::Duration(_delay_list_[i]), boost::bind(&TrajectoryLoader::publishTrajectory, this, _1, i), true,
                                            true));  // the last boolean argument makes the timer run only once);
  }

  /* timeout for threads */
  timeoutFunction();
}

//}

/* TrajectoryLoader::callMultipleServiceTriggers() //{ */

// Main method for calling Trigger service. This method creates independent thread for each calling.
void TrajectoryLoader::callMultipleServiceTriggers() {

  // | ------------------------ services ------------------------ |
  string             topic;
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
    thread_list_.push_back(_nh_.createTimer(ros::Duration(_delay_list_[i]), boost::bind(&TrajectoryLoader::callServiceTrigger, this, _1, i), true,
                                            true));  // the last boolean argument makes the timer run only once);
  }

  /* timeout for threads */
  timeoutFunction();
}

//}

/* TrajectoryLoader::timeoutFunction() //{ */

// Method that checks results from service threads and also checks if calling doesn't take too much time
void TrajectoryLoader::timeoutFunction() {
  std::vector<double>::iterator result;
  result         = std::max_element(_delay_list_.begin(), _delay_list_.end());
  double timeout = *result + _timeout_for_calling_services_;
  ROS_INFO("Main thread: Sleeping for %.1f before termination of all threads", timeout);
  const double timestep        = 0.1;
  double       current_timeout = 0;

  ros::AsyncSpinner spinner(_uav_name_list_.size());
  spinner.start();

  while (current_timeout < timeout) {
    ros::Duration(timestep).sleep();
    current_timeout += timestep;
    bool total_result = true;
    for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
      total_result = total_result && result_info_list_.at(i);
    }
    if (total_result) {
      ROS_INFO("Main thread: All service threads finished.");
      return;
    }
  }

  ROS_ERROR("Main thread: Timeout reached --> terminating all remaining threads");
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    if (!result_info_list_.at(i)) {
      ROS_ERROR("%s: Deadlock during calling. Shutting down the thread.", _uav_name_list_.at(i).c_str());
      thread_list_.at(i).stop();
    }
  }
}

//}

/* TrajectoryLoader::publishTrajectory() //{ */

// Method for publishing trajectory msg on specific service topic
void TrajectoryLoader::publishTrajectory([[maybe_unused]] const ros::TimerEvent &event, const int index) {
  ROS_INFO("%s: Calling service: %s", _uav_name_list_[index].c_str(), service_client_list_[index].getService().c_str());

  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = trajectories_list_[index];

  // calling service
  if (service_client_list_[index].call(srv)) {
    // when the calling was successful and the client received response
    if (srv.response.success) {
      if (srv.response.modified) {
        ROS_WARN("%s: %s", _uav_name_list_[index].c_str(), srv.response.message.c_str());
      } else {
        ROS_INFO("%s: %s", _uav_name_list_[index].c_str(), srv.response.message.c_str());
      }
    } else {
      ROS_ERROR("%s: %s", _uav_name_list_[index].c_str(), srv.response.message.c_str());
    }
    // service calling was unsuccessful because other side doesn't exist
  } else {
    ROS_ERROR("%s: Failed to call service", _uav_name_list_[index].c_str());
  }
  result_info_list_.at(index) = true;
}

//}

/* TrajectoryLoader::callServiceTrigger() //{ */

// Method for calling service msg Trigger on specific topic
void TrajectoryLoader::callServiceTrigger([[maybe_unused]] const ros::TimerEvent &event, const int index) {
  ROS_INFO("%s: Calling service: %s", _uav_name_list_[index].c_str(), service_client_list_[index].getService().c_str());

  std_srvs::Trigger srv;
  // calling service
  if (service_client_list_[index].call(srv)) {
    // when the calling was successful and the client received response
    if (srv.response.success) {
      ROS_INFO("%s: %s", _uav_name_list_[index].c_str(), srv.response.message.c_str());
    } else {
      ROS_ERROR("%s: %s", _uav_name_list_[index].c_str(), srv.response.message.c_str());
    }
    // service calling was unsuccessful because other side doesn't exist
  } else {
    ROS_ERROR("%s: Failed to call service", _uav_name_list_[index].c_str());
  }
  result_info_list_.at(index) = true;
}

//}

/* Auxiliary function for checking input for validity.//{ */

/* Function used to check that 'opt1' and 'opt2' are not specified at the same time. */
bool conflicting_options(const po::variables_map &vm, const char *opt1, const char *opt2) {
  if (vm.count(opt1) && !vm[opt1].defaulted() && vm.count(opt2) && !vm[opt2].defaulted()) {
    return true;
  }
  return false;
}

//}

/* MAIN FUNCTION //{ */
int main(int argc, char **argv) {
  /* Loading arguments //{ */

  po::options_description desc("Allowed options");

  // Declare the supported options.
  // clang-format off
  desc.add_options()
    ("help", "produce help message")
    ("load", "set flag for loading trajectories")
    ("call-trigger-service", "set flag for calling trigger service");
  // clang-format on

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc, po::command_line_style::unix_style), vm);
    po::notify(vm);
  }
  catch (std::exception &e) {
    std::cerr << e.what() << "\n";
    std::cout << desc << "\n";
    return 1;
  }

  // Call help() if it is specified or when the number of the arguments is less then one
  if (vm.count("help") || vm.size() == 0) {
    std::cout << desc << "\n";
    return 1;
  }

  // check if both arguments are not set
  if (conflicting_options(vm, "load", "call-trigger-service")) {
    std::cerr << "Conflicting options 'load' and 'call-trigger-service'. Only one of them can be set."
              << "\n";
    return 1;
  }

  //}

  if (vm.count("load")) {
    ROS_INFO("-------------- trajectory_loader - set for loading trajectories -----------------");
  } else {
    ROS_INFO("-------------- trajectory_loader - set for calling trigger service -----------------");
  }

  // initialize ROS
  ros::init(argc, argv, "trajectory_loader");
  ROS_INFO("Node initialized.");

  // create class TrajectoryLoader
  TrajectoryLoader trajectory_loader;

  // compare argument if it is equal to "--load"
  if (vm.count("load")) {
    trajectory_loader.loadMultipleTrajectories();
    // or "--call-trigger-service"
  } else if (vm.count("call-trigger-service")) {
    trajectory_loader.callMultipleServiceTriggers();
    // otherwise call help()
  } else {
    std::cout << desc << "\n";
    return 1;
  }
  ros::shutdown();
};

//}
