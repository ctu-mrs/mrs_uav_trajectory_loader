/* author: Vojtech Spurny*/

#include <ros/ros.h>
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
#include <iostream>
#include <fstream>

/**
 * @brief The TrajectoryLoader class
 */
class TrajectoryLoader {

public:

    TrajectoryLoader();                                                         // definition of constructor

private:
    mrs_msgs::TrackerTrajectory trajectory_;
    ros::NodeHandle nh_;
    
    // publishers
    ros::ServiceClient service_client_trajectory_;

		bool use_yaw_; // whether to fly trajectory with desired yaw
		bool fly_now_; // whether to start tracking the trajectory immediately after loading
		bool loop_;
		double max_altitude_, min_altitude_;
    std::string filename_;
    
    // functions
    void publishTrajectory();     
    void loadTrajectoryFromFile();
};

/**
 * @brief TrajectoryLoader::TrajectoryLoaders
 */
TrajectoryLoader::TrajectoryLoader() {

    nh_ = ros::NodeHandle("~");

    // publisher for new odometry
    service_client_trajectory_ = nh_.serviceClient<mrs_msgs::TrackerTrajectorySrv>("trajectory_topic");
		
		// load params
		use_yaw_ = nh_.param("use_yaw", false);
		fly_now_ = nh_.param("fly_now", false);
		loop_ = nh_.param("loop", false);
		filename_ = nh_.param("filename", std::string());
		min_altitude_ = nh_.param("min_altitude", 1.0);	
		max_altitude_ = nh_.param("max_altitude", 20.0);

    if (filename_.empty()){
        ROS_ERROR("The filename has to be set. \nFor example: rosrun trajectory_handler load_trajectory _filename:=trajectory.txt");
        ros::shutdown();
        return;
    }


    loadTrajectoryFromFile();
    publishTrajectory();
}


/**
 * @brief TrajectoryLoader::publishTrajectory
 */
void TrajectoryLoader::publishTrajectory() {
  // publish plan by service client
  mrs_msgs::TrackerTrajectorySrv srv;
  srv.request.trajectory_msg = trajectory_;
  ROS_INFO("publishing trajectory");
  if (service_client_trajectory_.call(srv)){
    // if calling was successful (mbzirc_tracker responded)
    if(srv.response.success){

      ROS_INFO("%s",srv.response.message.c_str());

    }else{

      ROS_ERROR("%s",srv.response.message.c_str());

    }
  }else {
  
      ROS_ERROR("%s",srv.response.message.c_str());
 
  }
}

/**
 * @brief TrajectoryLoader::loadTrajectoryFromFile
 */
void TrajectoryLoader::loadTrajectoryFromFile(){
    
    std::ifstream in(filename_.c_str(), std::ifstream::in);
    
    if (!in) {

       ROS_ERROR("Cannot open %s", filename_.c_str());
       ros::shutdown();
       return;
    
    } else {

      ROS_INFO("Loading from file: %s", filename_.c_str());
      std::string line;
      mrs_msgs::TrackerTrajectory new_traj;

      while (getline(in, line)) {
        mrs_msgs::TrackerPoint point;
        std::istringstream s(line);

        s >> point.x;
        s >> point.y;
        s >> point.z;
        s >> point.yaw;
        if (point.z > max_altitude_) {

          ROS_WARN("Selected altitude %2.2f saturated to max altitude %2.2f", point.z, max_altitude_);
          point.z = max_altitude_;

        } else if (point.z < min_altitude_) {

          ROS_WARN("Selected altitude %2.2f saturated to min altitude %2.2f", point.z, min_altitude_);
          point.z = min_altitude_;
        }

        new_traj.points.push_back(point);
      }
      in.close();

      new_traj.header.stamp = ros::Time::now();
      new_traj.use_yaw = use_yaw_;
      new_traj.fly_now = fly_now_;
      new_traj.loop = loop_;
      trajectory_ = new_traj;
    }
}

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv){

    ros::init(argc, argv, "trajectory_loader");
    ROS_INFO ("Node initialized.");

    TrajectoryLoader tl;
    return 0;
};
