#ifndef APF_H
#define APF_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <chrono>

struct node
{
	int idx;
	std::vector<double> pose;
	int parent;
	double cost;
};

struct sphere_obstacle
{
	std::vector<double> center;
	double radius;
};

struct rectangle_obstacle
{
	std::vector<double> center;
	double x_scale_;
	double y_scale_;
	double z_scale_;
};

class APF
{
public:
	APF();
	void spinner();
	void bringup();
	double euclideanDistance(std::vector<double> x1, std::vector<double> x2);
	void obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& data);
	void a1PoseCallback(const nav_msgs::Odometry::ConstPtr& data);
	void markerDeclaration(std::vector<double> x1, std::vector<double> x2);
private:
	std::vector<int> lim_;
	std::vector<rectangle_obstacle> obstacles_;
	ros::NodeHandle nh_;
	ros::Publisher marker_pub_;
	ros::Subscriber a1_pose_sub_, obs_sub_, goal_sub_;
	visualization_msgs::MarkerArray markers_;
	std::vector<double> x_start_;
	std::vector<double> x_goal_;
	std::vector<double> x_curr_;
	double distance_goal_;
	double gradient_goal_;
	bool goal_received_ = false;
	bool init_received_ = false;
	bool obs_1_received_ = false;
	bool obs_2_received_ = false;
};

#endif /* APF.H */
