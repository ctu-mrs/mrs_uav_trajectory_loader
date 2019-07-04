#ifndef TRAJECTORY_LOADER_H
#define TRAJECTORY_LOADER_H

#include <mrs_msgs/TrackerTrajectorySrv.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <boost/thread/thread.hpp>

class TrajectoryLoader {
private:
  ros::NodeHandle _nh_;

  /* trajectory parameters */
  std::string _current_working_directory_;
  bool        _use_yaw_;  // whether to fly trajectory with desired yaw
  bool        _fly_now_;  // whether to start tracking the trajectory immediately after loading
  bool        _loop_;     // whether to fly trajectory in loop (infinitely times)

  /* config parameters */
  double                   _timeout_for_calling_services_;
  std::string              _uav_name_;
  std::vector<double>      _offset_list_;
  std::vector<double>      _delay_list_;
  std::string              _service_topic_;
  std::vector<std::string> _uav_name_list_;

  std::vector<bool>                        result_info_list_;
  std::vector<mrs_msgs::TrackerTrajectory> _trajectories_list_;
  std::vector<ros::ServiceClient>          _service_client_list_;

  /* functions definition */
  void loadTrajectoryFromFile(const std::string &filename, mrs_msgs::TrackerTrajectory &trajectory);
  void publishTrajectory(const int index);
  void callServiceTrigger(const int index);
  void timeoutFunction();

  /* threads for asynchronous calling */
  boost::thread_group thread_group;

public:
  TrajectoryLoader();
  void loadMultipleTrajectories();
  void callMultipleServiceTriggers();
};
#endif
