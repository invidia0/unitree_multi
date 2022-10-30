#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Geometry>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

class hover{
  private:
    const float DEG_2_RAD = M_PI / 180.0;
    int getinput;
    double t, x, y, z, yaw;
    std::string input;
    bool isx, isy, isz;
    double desired_yaw = 0.0;
    int64_t time_from_start_ns = 0;

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    std::vector<WaypointWithTime> waypoints;
    static const int64_t kNanoSecondsInSecond = 1000000000;
    
  
  public:
    hover(){
      pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/crazyflie2/command/trajectory", 10);
      run();
      //sub = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10, &hover::callback, this);
    }
    void run(){
      while(ros::ok){
        std::cout << "Desired x [FLOAT] >> ";
        std::cin >> x;
        std::cout << "Desired y [FLOAT] >> ";
        std::cin >> y;
        std::cout << "Desired z [FLOAT] >> ";
        std::cin >> z;
        ROS_INFO("Received input: %f;%f;%f", x, y, z);
        if(double(x) == x && double(y) == y && double(z) == z){
          waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
          trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
          msg->header.stamp = ros::Time::now();
          msg->points.resize(waypoints.size());
          msg->joint_names.push_back("base_link");
          for (size_t i = 0; i < waypoints.size(); ++i) {
            WaypointWithTime& wp = waypoints[i];

            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = wp.position;
            trajectory_point.setFromYaw(wp.yaw);
            trajectory_point.time_from_start_ns = time_from_start_ns;

            time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
          }
          pub.publish(msg);
        }
        else{
          ROS_FATAL("Invalid input");
        }
        ros::spinOnce();
      }
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "hover_control");
  hover();
}