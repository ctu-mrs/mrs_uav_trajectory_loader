#include "trajectory_loader.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <boost/chrono/chrono_io.hpp>

using boost::lexical_cast;
using std::string;
namespace po = boost::program_options;

/* TrajectoryLoader::TrajectoryLoader() //{ */

TrajectoryLoader::TrajectoryLoader() {
  _nh_ = ros::NodeHandle("~");

  /* load parameters */
  _service_topic_ = _nh_.param("service_topic", string(""));
  if (_service_topic_.empty()) {
    ROS_ERROR("Parameter service_topic is not set!");
    return;
  }

  _uav_name_ = _nh_.param("uav_name", string(""));
  _nh_.getParam("main/uav_name_list", _uav_name_list_);
  if (_uav_name_list_.empty()) {
    if (_uav_name_.empty()) {
      ROS_ERROR("uav_name_list (target UAVs) is empty!");
      return;
    } else {
      ROS_INFO("uav_name_list (target UAVs) is empty -> setting the list to: [%s]", _uav_name_.c_str());
      _uav_name_list_.push_back(_uav_name_);
    }
  }

  _nh_.getParam("main/delay", _delay_list_);
  if (_delay_list_.size() != _uav_name_list_.size()) {
    ROS_ERROR("delay_list (parameter main/delay) should have the same size as uav_name_list!");
    return;
  }

  // check if the delays are positive values and if some delays are set
  double delay_sum = 0;
  for (unsigned long i = 0; i < _delay_list_.size(); i++) {
    delay_sum += _delay_list_.at(i);
    if (_delay_list_.at(i) < 0) {
      ROS_ERROR("delay has to be positive value!");
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

  _timeout_for_calling_services_ = _nh_.param("timeout_for_calling_services", double(5));
  if (_timeout_for_calling_services_ < 0) {
    ROS_ERROR("Parameter timeout_for_calling_services has to be positive value!");
    return;
  }
}

//}

/* TrajectoryLoader::loadTrajectoryFromFile //{ */

// method for loading trajectories from file into mrs_msgs/TrackerTrajectory msg
void TrajectoryLoader::loadTrajectoryFromFile(const string &filename, mrs_msgs::TrackerTrajectory &trajectory) {
  std::ifstream in(filename.c_str(), std::ifstream::in);

  if (!in) {
    ROS_ERROR("- Cannot open %s", filename.c_str());
    return;

  } else {

    ROS_INFO("- Loading trajectory from %s", filename.c_str());

    string                      line;
    mrs_msgs::TrackerTrajectory new_traj;
    mrs_msgs::TrackerPoint      point;
    std::vector<string>         parts;

    /* for each line in file */
    while (getline(in, line)) {
      // replace comma with space
      boost::replace_all(line, ",", " ");
      // if now line contains two space replace them with one space
      boost::replace_all(line, "  ", " ");
      // split string using space delimiter
      boost::split(parts, line, boost::is_any_of(" "));

      if (parts.size() != 4) {
        ROS_ERROR("- Incorrect format of the trajectory on line %d", int(new_traj.points.size() + 1));
        return;
      }

      try {
        point.x   = lexical_cast<double>(parts[0]);
        point.y   = lexical_cast<double>(parts[1]);
        point.z   = lexical_cast<double>(parts[2]);
        point.yaw = lexical_cast<double>(parts[3]);
      }
      catch (...) {
        ROS_ERROR("- Some error occured during reading line %d", int(new_traj.points.size() + 1));
        return;
      }

      point.x += _offset_list_[0];
      point.y += _offset_list_[1];
      point.z += _offset_list_[2];
      point.yaw += _offset_list_[3];

      new_traj.points.push_back(point);
    }
    in.close();

    new_traj.header.stamp = ros::Time::now();
    new_traj.use_yaw      = _use_yaw_;
    new_traj.fly_now      = _fly_now_;
    new_traj.loop         = _loop_;
    trajectory            = new_traj;
  }
}

//}

/* TrajectoryLoader::loadMultipleTrajectories() //{ */

// Main method for loading trajectories. This method creates independent thread for each uav.
void TrajectoryLoader::loadMultipleTrajectories() {
  /* get parameters */
  _use_yaw_ = _nh_.param("use_yaw", false);
  _fly_now_ = _nh_.param("fly_now", false);
  _loop_    = _nh_.param("loop", false);

  ROS_INFO("Trajectory setting parameters: FLY_NOW = %s, USE_YAW = %s, LOOP = %s", _fly_now_ ? "True" : "False", _use_yaw_ ? "True" : "False",
           _loop_ ? "True" : "False");

  _nh_.getParam("main/offset", _offset_list_);
  if (_offset_list_.size() != 4) {
    ROS_ERROR("Parameter offset should be set to [x,y,z,yaw]!");
    return;
  }

  double offset_sum = _offset_list_[0] + _offset_list_[1] + _offset_list_[2] + _offset_list_[3];
  if (offset_sum > 1e-5) {
    ROS_WARN("Trajectories will be offsetted by [%.2f, %.2f, %.2f, %.2f]", _offset_list_[0], _offset_list_[1], _offset_list_[2], _offset_list_[3]);
  }

  _current_working_directory_ = _nh_.param("current_working_directory", string(""));
  if (_current_working_directory_.empty()) {
    ROS_ERROR("Parameter current_working_directory is not set!");
    return;
  }

  string text;
  string filename_array[_uav_name_list_.size()];
  if (_uav_name_list_.size() != 1 && _uav_name_ != _uav_name_list_[0]) {
    for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
      text            = _uav_name_list_[i] + "/filename";
      string filename = _nh_.param(text.c_str(), string(""));
      if (filename.empty()) {
        ROS_ERROR("Parameter %s is not set in config file!!", text.c_str());
        return;
      }

      filename_array[i] = _current_working_directory_ + filename;
    }
  } else {
    text = _nh_.param("filename", string(""));
    if (text.empty()) {
      ROS_ERROR("Parameter filename is not set in config file!!");
      return;
    }
    filename_array[0] = _current_working_directory_ + text;
  }

  /* load trajectories */
  ROS_INFO("Loading trajectories from files:");
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    mrs_msgs::TrackerTrajectory msg;
    loadTrajectoryFromFile(filename_array[i], msg);
    _trajectories_list_.push_back(msg);
  }

  ROS_INFO("Trajectories successfully loaded into internal array.\n");

  /* create services */
  string             topic;
  ros::ServiceClient sc;
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    topic = "/" + _uav_name_list_[i] + "/" + _service_topic_;
    sc    = _nh_.serviceClient<mrs_msgs::TrackerTrajectorySrv>(topic.c_str());
    _service_client_list_.push_back(sc);
    result_info_list_.push_back(false);
  }

  /* call services */
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    boost::thread *t = new boost::thread(&TrajectoryLoader::publishTrajectory, this, i);
    thread_group.add_thread(t);
  }

  /* timeout for threads */
  timeoutFunction();

  thread_group.interrupt_all();
}

//}

/* TrajectoryLoader::callMultipleServiceTriggers() //{ */

// Main method for calling Trigger service. This method creates independent thread for each calling.
void TrajectoryLoader::callMultipleServiceTriggers() {
  /* create services */
  string             topic;
  ros::ServiceClient sc;
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    topic = "/" + _uav_name_list_[i] + "/" + _service_topic_;
    sc    = _nh_.serviceClient<std_srvs::Trigger>(topic.c_str());
    _service_client_list_.push_back(sc);
    result_info_list_.push_back(false);
  }

  /* call services */
  for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
    boost::thread *t = new boost::thread(&TrajectoryLoader::callServiceTrigger, this, i);
    thread_group.add_thread(t);
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
  while (current_timeout < timeout) {
    ros::Duration(timestep).sleep();
    current_timeout += timestep;
    bool total_result = true;
    for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
      total_result &= result_info_list_.at(i);
    }
    if (total_result) {
      ROS_INFO("Main thread: All service threads finished.");
      break;
    }
  }

  if (current_timeout >= timeout) {
    ROS_ERROR("Main thread: Timeout reached --> terminating all remaining threads");
    for (unsigned long i = 0; i < _uav_name_list_.size(); ++i) {
      if (!result_info_list_.at(i)) {
        ROS_ERROR("%s: Deadlock during calling", _uav_name_list_.at(i).c_str());
      }
    }
  }
}

//}

/* TrajectoryLoader::publishTrajectory() //{ */

// Method for publishing trajectory msg on specific service topic
void TrajectoryLoader::publishTrajectory(const int index) {
  // sleep if delay is set
  if (_delay_list_[index] > 1e-3) {
    ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", _uav_name_list_[index].c_str(), _delay_list_[index],
             _service_client_list_[index].getService().c_str());
    ros::Duration(_delay_list_[index]).sleep();
  }

  ROS_INFO("%s: Calling service: %s", _uav_name_list_[index].c_str(), _service_client_list_[index].getService().c_str());

  mrs_msgs::TrackerTrajectorySrv srv;
  srv.request.trajectory_msg = _trajectories_list_[index];

  // calling service
  if (_service_client_list_[index].call(srv)) {
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

/* TrajectoryLoader::callServiceTrigger() //{ */

// Method for calling service msg Trigger on specific topic
void TrajectoryLoader::callServiceTrigger(const int index) {
  // sleep if delay is set
  if (_delay_list_[index] > 1e-3) {
    ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", _uav_name_list_[index].c_str(), _delay_list_[index],
             _service_client_list_[index].getService().c_str());
    ros::Duration(_delay_list_[index]).sleep();
  }

  ROS_INFO("%s: Calling service: %s", _uav_name_list_[index].c_str(), _service_client_list_[index].getService().c_str());

  std_srvs::Trigger srv;
  // calling service
  if (_service_client_list_[index].call(srv)) {
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

  // initialize ROS
  ros::init(argc, argv, "trajectory_loader");
  ROS_INFO("Node initialized.");

  // create class TrajectoryLoader
  TrajectoryLoader trajectory_loader;

  // compare argument if it is equal to "--load"
  if (vm.count("load")) {

    ROS_INFO("LOADING ....");
    trajectory_loader.loadMultipleTrajectories();

    // or "--call-trigger-service"
  } else if (vm.count("call-trigger-service")) {

    ROS_INFO("CALLING TRIGGER SERVICE ....");
    trajectory_loader.callMultipleServiceTriggers();

    // otherwise call help()
  } else {
    std::cout << desc << "\n";
    return 1;
  }
};

//}