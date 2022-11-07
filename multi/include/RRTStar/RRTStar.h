#ifndef RRTSTAR_H
#define RRTSTAR_H

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

class RRTStar
{
public:
	RRTStar();
	void spinner(void);
	void bringup();
	void initialize();
	void plotPoint();
	void plotObstacles();
	void plotLine(std::vector<double> x1, std::vector<double> x2, std::vector<double> c = {0, 0.2, 0.8}, double w = 1);
	void generate_path();
	void bezier_curve_generator();
	void send_path();
	void add_final_path(geometry_msgs::Point curr, geometry_msgs::Point next);
	std::vector<double> randomPoint();
	double euclideanDistance(std::vector<double> x1, std::vector<double> x2);
	node closest(std::vector<double> x);
	std::vector<node> nearestSet(node new_node);
	bool intersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);
	bool checkRectangleIntersection(std::vector<double> x1, std::vector<double> x2);
	bool feasible(node possible_node);
	bool feasible(node node1, node node2);
	void cheapest(node &new_node, std::vector<node> nearest_set);
	void rewiring(node new_node, std::vector<node> nearest_set);
	void expandGraph(std::vector<double> x_r, int goal = 0);
	void checkEnd();
	void solve();
	void obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& data);
	void a1PoseCallback(const nav_msgs::Odometry::ConstPtr& data);
	bool goal_feasible();
private:
	std::vector<geometry_msgs::Point> smoothed_path_;
	std::vector<int> lim_;
	std::vector<rectangle_obstacle> obstacles_;
	ros::NodeHandle nh_;
	ros::Publisher marker_pub_, path_pub_;
	ros::Subscriber a1_pose_sub_, obs_sub_, goal_sub_;
	visualization_msgs::MarkerArray markers_;
	visualization_msgs::Marker marker_, final_edge_;
	std::vector<node> graph_;
	std::vector<double> x_start_;
	std::vector<double> x_goal_;
	std::vector<node> path_;
	double epsilon_;
	double d_;
	double mu_free_;
	double z_d_;
	int it_max_;
	bool finish_;
	int it_;
	bool goal_received_ = false;
	bool init_received_ = false;
	bool obs_1_received_ = false;
	bool obs_2_received_ = false;
	bool plots_ = false;
};

#endif /* RRTSTAR_H */
