#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

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

class RRTStar
{
public:
	RRTStar();
	RRTStar(std::vector<double> x_start, std::vector<double> x_goal, double epsilon, double d, std::vector<int> lim, std::vector<sphere_obstacle> obstacles, int it_max);
	void spinner(void);

private:
	void bringup();
	void initialize();
	void plotPoint(std::vector<double> x, std::vector<double> c = {1, 0.5, 0}, double s = 30);
	void plotObstacles();
	void plotLine(std::vector<double> x1, std::vector<double> x2, std::vector<double> c = {0, 0.2, 0.8}, double w = 1.5);
	void plotPath();
	std::vector<double> randomPoint();
	double euclideanDistance(std::vector<double> x1, std::vector<double> x2);
	node closest(std::vector<double> x);
	std::vector<node> nearestSet(node new_node);
	int checkSphereIntersection(std::vector<double> x1, std::vector<double> x2, sphere_obstacle o);
	int checkCollision(node possible_node);
	int checkCollision(node node1, node node2);
	void cheapest(node &new_node, std::vector<node> nearest_set);
	void rewiring(node new_node, std::vector<node> nearest_set);
	void expandGraph(std::vector<double> x_r, int goal = 0);
	void checkEnd();
	void solve();
	ros::NodeHandle nh_;
	ros::Publisher marker_pub_;
	ros::Subscriber a1_pose_sub_, obs_sub_, goal_sub_;
	visualization_msgs::MarkerArray markers_;
	std::vector<node> graph_;
	std::vector<double> x_start_;
	std::vector<double> x_goal_;
	double epsilon_;
	double d_;
	double mu_free_;
	double z_d_;
	std::vector<int> lim_;
	std::vector<sphere_obstacle> obstacles_;
	int it_max_;
	bool finish_;
	int it_;
};

#endif /* RRTSTAR_H */
