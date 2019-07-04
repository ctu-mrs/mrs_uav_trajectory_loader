/* author: Vojtech Spurny*/

#include <mrs_msgs/TrackerTrajectorySrv.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>

#include <fstream>
#include <iostream>

#include <algorithm>

#include <boost/chrono/chrono_io.hpp>

using boost::lexical_cast;
using std::string;
namespace po = boost::program_options;


class TrajectoryLoader {
public:
  TrajectoryLoader();  // definition of constructor
  void loadMultipleTrajectories();
  void callMultipleServiceTriggers();

private:
  ros::NodeHandle nh_;
  string          current_working_directory_;

  /* trajectory parameters */
  bool                use_yaw_;  // whether to fly trajectory with desired yaw
  bool                fly_now_;  // whether to start tracking the trajectory immediately after loading
  bool                loop_;
  double              timeout_for_calling_services_;
  std::vector<double> offset_list_;
  std::vector<double> delay_list_;
  string              uav_name_;

  /* config parameters */
  std::vector<string>                      uav_name_list_;
  std::vector<bool>                        result_info_list_;
  std::vector<mrs_msgs::TrackerTrajectory> trajectories_list_;
  std::vector<ros::ServiceClient>          service_client_list_;
  string                                   service_topic_;

  /* functions definition */
  void loadTrajectoryFromFile(const string &filename, mrs_msgs::TrackerTrajectory &trajectory);
  void publishTrajectory(const int index);
  void callServiceTrigger(const int index);
  void timeoutFunction();

  /* threads for asynchronous calling */
  boost::thread_group thread_group;
};

// constructor
TrajectoryLoader::TrajectoryLoader() {
  nh_ = ros::NodeHandle("~");

  /* load parameters */
  service_topic_ = nh_.param("service_topic", string(""));
  if (service_topic_.empty()) {
    ROS_ERROR("Parameter service_topic is not set!");
    ros::shutdown();
    return;
  }

  uav_name_ = nh_.param("uav_name", string(""));
  nh_.getParam("main/uav_name_list", uav_name_list_);
  if (uav_name_list_.empty()) {
    if (uav_name_.empty()) {
      ROS_ERROR("uav_name_list (target UAVs) is empty!");
      ros::shutdown();
      return;
    } else {
      ROS_INFO("uav_name_list (target UAVs) is empty -> setting the list to: [%s]", uav_name_.c_str());
      uav_name_list_.push_back(uav_name_);
    }
  }

  nh_.getParam("main/delay", delay_list_);
  if (delay_list_.size() != uav_name_list_.size()) {
    ROS_ERROR("delay_list (parameter main/delay) should have the same size as uav_name_list!");
    ros::shutdown();
    return;
  }

  // check if the delays are positive values and if some delays are set
  double delay_sum = 0;
  for (unsigned long i = 0; i < delay_list_.size(); i++) {
    delay_sum += delay_list_.at(i);
    if (delay_list_.at(i) < 0) {
      ROS_ERROR("delay has to be positive value!");
      ros::shutdown();
      return;
    }
  }

  char buff[100];
  if (delay_sum > 1e-5) {
    string delay_text = "Delays are set to: [ ";
    for (unsigned long i = 0; i < delay_list_.size(); i++) {
      snprintf(buff, sizeof(buff), "%s: %.1f", uav_name_list_[i].c_str(), delay_list_[i]);
      delay_text += buff;
      if (i != delay_list_.size() - 1) {
        delay_text += ", ";
      }
    }
    delay_text += "]";
    ROS_WARN("%s", delay_text.c_str());
  }

  timeout_for_calling_services_ = nh_.param("timeout_for_calling_services", double(5));
  if (timeout_for_calling_services_ < 0) {
    ROS_ERROR("Parameter timeout_for_calling_services has to be positive value!");
    ros::shutdown();
    return;
  }
}

// method for loading trajectories from file into mrs_msgs/TrackerTrajectory msg
void TrajectoryLoader::loadTrajectoryFromFile(const string &filename, mrs_msgs::TrackerTrajectory &trajectory) {
  std::ifstream in(filename.c_str(), std::ifstream::in);

  if (!in) {
    ROS_ERROR("- Cannot open %s", filename.c_str());
    ros::shutdown();
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
        ros::shutdown();
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
        ros::shutdown();
        return;
      }

      point.x += offset_list_[0];
      point.y += offset_list_[1];
      point.z += offset_list_[2];
      point.yaw += offset_list_[3];

      new_traj.points.push_back(point);
    }
    in.close();

    new_traj.header.stamp = ros::Time::now();
    new_traj.use_yaw      = use_yaw_;
    new_traj.fly_now      = fly_now_;
    new_traj.loop         = loop_;
    trajectory            = new_traj;
  }
}

// Main method for loading trajectories. This method creates independent thread for each uav.
void TrajectoryLoader::loadMultipleTrajectories() {
  /* get parameters */
  use_yaw_ = nh_.param("use_yaw", false);
  fly_now_ = nh_.param("fly_now", false);
  loop_    = nh_.param("loop", false);

  ROS_INFO("Trajectory setting parameters: FLY_NOW = %s, USE_YAW = %s, LOOP = %s", fly_now_ ? "True" : "False", use_yaw_ ? "True" : "False",
           loop_ ? "True" : "False");

  nh_.getParam("main/offset", offset_list_);
  if (offset_list_.size() != 4) {
    ROS_ERROR("Parameter offset should be set to [x,y,z,yaw]!");
    ros::shutdown();
    return;
  }

  double offset_sum = offset_list_[0] + offset_list_[1] + offset_list_[2] + offset_list_[3];
  if (offset_sum > 1e-5) {
    ROS_WARN("Trajectories will be offsetted by [%.2f, %.2f, %.2f, %.2f]", offset_list_[0], offset_list_[1], offset_list_[2], offset_list_[3]);
  }

  current_working_directory_ = nh_.param("current_working_directory", string(""));
  if (current_working_directory_.empty()) {
    ROS_ERROR("Parameter current_working_directory is not set!");
    ros::shutdown();
    return;
  }

  string text;
  string filename_array[uav_name_list_.size()];
  if (uav_name_list_.size() != 1 && uav_name_ != uav_name_list_[0]) {
    for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
      text            = uav_name_list_[i] + "/filename";
      string filename = nh_.param(text.c_str(), string(""));
      if (filename.empty()) {
        ROS_ERROR("Parameter %s is not set in config file!!", text.c_str());
        ros::shutdown();
        return;
      }

      filename_array[i] = current_working_directory_ + filename;
    }
  } else {
    text = nh_.param("filename", string(""));
    if (text.empty()) {
      ROS_ERROR("Parameter filename is not set in config file!!");
      ros::shutdown();
      return;
    }
    filename_array[0] = current_working_directory_ + text;
  }

  /* load trajectories */
  ROS_INFO("Loading trajectories from files:");
  for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
    mrs_msgs::TrackerTrajectory msg;
    loadTrajectoryFromFile(filename_array[i], msg);
    trajectories_list_.push_back(msg);
  }

  ROS_INFO("Trajectories successfully loaded into internal array.\n");

  /* create services */
  string             topic;
  ros::ServiceClient sc;
  for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
    topic = "/" + uav_name_list_[i] + "/" + service_topic_;
    sc    = nh_.serviceClient<mrs_msgs::TrackerTrajectorySrv>(topic.c_str());
    service_client_list_.push_back(sc);
    result_info_list_.push_back(false);
  }

  /* call services */
  for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
    boost::thread *t = new boost::thread(&TrajectoryLoader::publishTrajectory, this, i);
    thread_group.add_thread(t);
  }

  /* timeout for threads */
  timeoutFunction();

  thread_group.interrupt_all();
  ros::shutdown();
}

// Main method for calling Trigger service. This method creates independent thread for each calling.
void TrajectoryLoader::callMultipleServiceTriggers() {
  /* create services */
  string             topic;
  ros::ServiceClient sc;
  for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
    topic = "/" + uav_name_list_[i] + "/" + service_topic_;
    sc    = nh_.serviceClient<std_srvs::Trigger>(topic.c_str());
    service_client_list_.push_back(sc);
    result_info_list_.push_back(false);
  }

  /* call services */
  for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
    boost::thread *t = new boost::thread(&TrajectoryLoader::callServiceTrigger, this, i);
    thread_group.add_thread(t);
  }

  /* timeout for threads */
  timeoutFunction();
  ros::shutdown();
}

// Method that checks results from service threads and also checks if calling didn't take too much time
void TrajectoryLoader::timeoutFunction() {
  std::vector<double>::iterator result;
  result         = std::max_element(delay_list_.begin(), delay_list_.end());
  double timeout = *result + timeout_for_calling_services_;
  ROS_INFO("Main thread: Sleeping for %.1f before termination of all threads", timeout);
  const double timestep        = 0.1;
  double       current_timeout = 0;
  while (current_timeout < timeout) {
    ros::Duration(timestep).sleep();
    current_timeout += timestep;
    bool total_result = true;
    for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
      total_result &= result_info_list_.at(i);
    }
    if (total_result) {
      ROS_INFO("Main thread: All service threads finished.");
      break;
    }
  }

  if (current_timeout >= timeout) {
    ROS_ERROR("Main thread: Timeout reached --> terminating all remaining threads");
    for (unsigned long i = 0; i < uav_name_list_.size(); ++i) {
      if (!result_info_list_.at(i)) {
        ROS_ERROR("%s: Deadlock during calling", uav_name_list_.at(i).c_str());
      }
    }
  }
}


// Method for publishing trajectory msg on specific service topic
void TrajectoryLoader::publishTrajectory(const int index) {
  // sleep if delay is set
  if (delay_list_[index] > 1e-3) {
    ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", uav_name_list_[index].c_str(), delay_list_[index],
             service_client_list_[index].getService().c_str());
    ros::Duration(delay_list_[index]).sleep();
  }

  ROS_INFO("%s: Calling service: %s", uav_name_list_[index].c_str(), service_client_list_[index].getService().c_str());

  mrs_msgs::TrackerTrajectorySrv srv;
  srv.request.trajectory_msg = trajectories_list_[index];

  // calling service
  if (service_client_list_[index].call(srv)) {
    // when the calling was successful and the client received response
    if (srv.response.success) {
      ROS_INFO("%s: %s", uav_name_list_[index].c_str(), srv.response.message.c_str());
    } else {
      ROS_ERROR("%s: %s", uav_name_list_[index].c_str(), srv.response.message.c_str());
    }
    // service calling was unsuccessful because other side doesn't exist
  } else {
    ROS_ERROR("%s: Failed to call service", uav_name_list_[index].c_str());
  }
  result_info_list_.at(index) = true;
}


// Method for calling service msg Trigger on specific topic
void TrajectoryLoader::callServiceTrigger(const int index) {
  // sleep if delay is set
  if (delay_list_[index] > 1e-3) {
    ROS_INFO("%s: Sleeping for %.1f s before calling service \"%s\"", uav_name_list_[index].c_str(), delay_list_[index],
             service_client_list_[index].getService().c_str());
    ros::Duration(delay_list_[index]).sleep();
  }

  ROS_INFO("%s: Calling service: %s", uav_name_list_[index].c_str(), service_client_list_[index].getService().c_str());

  std_srvs::Trigger srv;
  // calling service
  if (service_client_list_[index].call(srv)) {
    // when the calling was successful and the client received response
    if (srv.response.success) {
      ROS_INFO("%s: %s", uav_name_list_[index].c_str(), srv.response.message.c_str());
    } else {
      ROS_ERROR("%s: %s", uav_name_list_[index].c_str(), srv.response.message.c_str());
    }
    // service calling was unsuccessful because other side doesn't exist
  } else {
    ROS_ERROR("%s: Failed to call service", uav_name_list_[index].c_str());
  }
  result_info_list_.at(index) = true;
}


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
  desc.add_options()("help", "produce help message")("load", "set flag for loading trajectories")("call-trigger-service",
                                                                                                  "set flag for calling trigger service");
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
  if (conflicting_options(vm, "load", "call-trigger-service")){
    std::cerr << "Conflicting options 'load' and 'call-trigger-service'. Only one of them can be set." << "\n";
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
