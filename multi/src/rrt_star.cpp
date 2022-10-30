/*

===============================================================================
TO DO:
1. Start point must be taken from the robot position
2. Goal point must be taken from the user input on RVIZ (click on the map)
3. Obstacles must be taken from the map
4. Make a collision check function for a generic obstacle not only spherical
5. Make a function to check if the goal is reachable from the start point with it_max_ iterations
===============================================================================

*/
#include "RRTStar/RRTStar.h"

RRTStar::RRTStar()
{
	bringup();
	/*
	Points are vectors! The first element is the x coordinate, the second is the y coordinate,
	and the third is the z coordinate.
	*/

	// initialize the x_start_ = {x, y, z}; vector
	x_start_ = {-8, -7, -9};

	// x_goal_ = {x, y, z};
	x_goal_ = {9, 5, 8};

	// epsilon_ = distance between nodes
	epsilon_ = 1.0;

	// d_ = distance between nodes in the path
	d_ = 10.0;

	// n_ob = number of obstacles
	int n_ob = 1;
	double r_max = 1.0;

	// Map limits
	int xmin = -10;
	int xmax = 10;
	int ymin = -10;
	int ymax = 10;
	int zmin = -10;
	int zmax = 10;

	// Map limits vector
	lim_ = {xmin, xmax, ymin, ymax, zmin, zmax};

	// Random generator
	srand(time(NULL));

	// Obstacles vector clearup
	obstacles_.clear();

	// Generate random SPHERE obstacles.
	while (obstacles_.size() < n_ob && ros::ok())
	{
		sphere_obstacle new_ob;
		new_ob.radius = (double)random() / RAND_MAX * r_max;
		new_ob.center = randomPoint();
		if (euclideanDistance(new_ob.center, x_start_) > new_ob.radius && euclideanDistance(new_ob.center, x_goal_) > new_ob.radius)
		{
			obstacles_.push_back(new_ob);
		}
	}

	// Max number of iterations.
	it_max_ = 500;

	initialize();
}

void RRTStar::bringup()
{
	a1_pose_sub_ = nh_.subscribe("/a1/pose", 1, &RRTStar::a1PoseCallback, this);
	obs_sub_ = nh_.subscribe("/a1/obstacles", 1, &RRTStar::obstaclesCallback, this);
	goal_sub_ = nh_.subscribe("/a1/goal", 1, &RRTStar::goalCallback, this);


}

RRTStar::RRTStar(std::vector<double> x_start, std::vector<double> x_goal, double epsilon, double d, std::vector<int> lim, std::vector<sphere_obstacle> obstacles, int it_max)
{
	x_start_ = x_start;
	x_goal_ = x_goal;
	epsilon_ = epsilon;
	d_ = d;
	lim_ = lim;
	obstacles_ = obstacles;
	it_max_ = it_max;
	initialize();
}

void RRTStar::initialize()
{
	// Publisher for the tree graph
	marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/RRTStar_graph", 1);
	ros::Duration(1).sleep();

	// Clear the markers
	markers_.markers.clear();

	// Add the start node to the tree
	plotPoint(x_start_, {0, 0.5, 0});

	// Add the goal node to the tree
	plotPoint(x_goal_, {0.4, 0.4, 0.6});

	// Plot the obstacles on the map
	plotObstacles();

	// This should be improved
	mu_free_ = 1.;
	// Free space definition
	for (uint i = 0; i < 3; i++)
	{
		mu_free_ *= (std::abs(lim_[2 * i]) + std::abs(lim_[2 * i + 1]));
	}
	// Subtract the obstacles volume from the free space
	for (uint i = 0; i < obstacles_.size(); i++)
	{
		mu_free_ -= obstacles_[i].radius * 4. / 3. * M_PI;
	}

	z_d_ = 1.0;

	// iteration counter
	it_ = 0;

	// Tree initialization
	// First node is the start node
	node first;
	first.pose = x_start_;
	first.parent = -1;
	first.idx = 0;
	first.cost = 0.0;
	graph_.push_back(first); // Push the first node to the tree

	finish_ = false;

	// How many iterations
	std::cout << "Insert maximum iteration number" << std::endl;
	std::cin >> it_max_;

	// Solve the graph
	solve();
}

void RRTStar::plotPoint(std::vector<double> x, std::vector<double> c, double s)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.ns = "RRTStar";
	marker.id = markers_.markers.size();
	marker.type = marker.SPHERE;
	marker.action = marker.ADD;
	marker.scale.x = s * 0.01;
	marker.scale.y = s * 0.01;
	marker.scale.z = s * 0.01;
	marker.color.a = 1.0;
	marker.color.r = c[0];
	marker.color.g = c[1];
	marker.color.b = c[2];
	marker.pose.orientation.w = 1.0;
	marker.pose.position.x = x[0];
	marker.pose.position.y = x[1];
	marker.pose.position.z = x[2];
	markers_.markers.push_back(marker);
	marker_pub_.publish(markers_);
}

void RRTStar::plotObstacles()
{
	for (auto &x : obstacles_)
		plotPoint(x.center, {0, 0.75, 0.75}, x.radius / 0.01);
}

void RRTStar::plotLine(std::vector<double> x1, std::vector<double> x2, std::vector<double> c, double w)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.ns = "RRTStar";
	marker.id = markers_.markers.size();
	marker.type = marker.LINE_LIST;
	marker.action = marker.ADD;
	marker.scale.x = w * 0.1;
	marker.color.a = 1.0;
	marker.color.r = c[0];
	marker.color.g = c[1];
	marker.color.b = c[2];

	geometry_msgs::Point x;
	x.x = x1[0];
	x.y = x1[1];
	x.z = x1[2];
	marker.points.push_back(x);

	x.x = x2[0];
	x.y = x2[1];
	x.z = x2[2];
	marker.points.push_back(x);

	marker.pose.orientation.w = 1.0;
	markers_.markers.push_back(marker);
	marker_pub_.publish(markers_);
}

void RRTStar::plotPath()
{
	// Plot the final path if it exists
	int idx = graph_.size() - 1;
	double total_cost = 0.;
	while (idx > 0 && ros::ok())
	{
		node child = graph_[idx];
		node parent = graph_[child.parent];
		total_cost += euclideanDistance(child.pose, parent.pose);
		plotLine(child.pose, parent.pose, {1, 0, 0}, 3);
		idx = parent.idx;
	}
	graph_.back().cost = total_cost;
}

std::vector<double> RRTStar::randomPoint()
{
	// Generate a random point in the free space
	std::vector<double> x_r;
	for (uint i = 0; i < x_start_.size(); i++)
	{
		double a = lim_[2 * i] + .0;
		double b = lim_[2 * i + 1] + .0;
		double r = (b - a) * rand() / RAND_MAX + a;
		x_r.push_back(r);
	}
	return x_r;
}

double RRTStar::euclideanDistance(std::vector<double> x1, std::vector<double> x2)
{
	double quad_dist = 0.0;
	for (uint i = 0; i < x1.size(); i++)
	{
		quad_dist += std::pow(x1[i] - x2[i], 2);
	}
	return std::sqrt(quad_dist);
}

node RRTStar::closest(std::vector<double> x)
{
	// Find the closest node in the tree to the random point with the euclidean distance
	node closest_node;
	double dist = 1000;
	for (auto &n : graph_)
	{
		double norm = euclideanDistance(x, n.pose);
		if (norm < dist)
		{
			dist = norm;
			closest_node = n;
		}
	}
	return closest_node;
}

std::vector<node> RRTStar::nearestSet(node new_node)
{
	std::vector<node> nearest_set;
	double gamma = 2 * std::pow(1. + 1. / d_, 1. / d_) * std::pow(mu_free_ / z_d_, 1. / d_);
	double n = graph_.size() + .0;

	double ball_radius = std::min(epsilon_, gamma * std::pow(std::log(n) / n, 1. / d_));
	for (auto &n : graph_)
	{
		double norm = euclideanDistance(new_node.pose, n.pose);
		if (norm < 2 * ball_radius)
		{
			nearest_set.push_back(n);
		}
	}
	return nearest_set;
}

int RRTStar::checkSphereIntersection(std::vector<double> x1, std::vector<double> x2, sphere_obstacle o)
{
	Eigen::Array3d p1, p2, c;
	for (uint i = 0; i < x1.size(); i++)
	{
		p1(i) = x1[i];
		p2(i) = x2[i];
		c(i) = o.center[i];
	}
	Eigen::Array3d a = c - p1;
	Eigen::Array3d b = p2 - p1;

	double u = 0.0;
	if (b.sum() != 0)
	{
		double u = (a * b).sum() / b.pow(2).sum();
		if (u < 0)
		{
			u = 0.0;
		}
		else if (u > 1)
		{
			u = 1.0;
		}
	}

	std::vector<double> p(x1.size());
	for (uint i = 0; i < p.size(); i++)
	{
		p[i] = p1(i) + u * b(i);
	}
	if (euclideanDistance(p, o.center) > o.radius)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

int RRTStar::checkCollision(node possible_node)
{
	// Check if the new node is in collision with the obstacles
	int inter = 0;
	node parent = graph_[possible_node.parent];
	for (auto &o : obstacles_)
	{
		inter = checkSphereIntersection(possible_node.pose, parent.pose, o);
		if (inter)
			break;
	}
	return inter;
}

int RRTStar::checkCollision(node node1, node node2)
{
	// Check if the new node is in collision with the obstacles
	int inter = 0;
	for (auto &o : obstacles_)
	{
		inter = checkSphereIntersection(node1.pose, node2.pose, o);
		if (inter)
			break;
	}
	return inter;
}

void RRTStar::cheapest(node &new_node, std::vector<node> nearest_set)
{
	// Find the cheapest node in the nearest set
	for (auto &n : nearest_set)
	{
		double norm = euclideanDistance(new_node.pose, n.pose);
		if ((n.cost + norm) < new_node.cost && !checkCollision(new_node, n))
		{
			new_node.cost = n.cost + norm;
			new_node.parent = n.idx;
		}
	}
}

void RRTStar::rewiring(node new_node, std::vector<node> nearest_set)
{
	// Rewtire the tree in the neighborhood of the new node to minimize the cost.
	for (auto &n : nearest_set)
	{
		double norm = euclideanDistance(new_node.pose, n.pose);
		if ((new_node.cost + norm) < n.cost && !checkCollision(new_node, n))
		{
			int idx = n.idx;
			graph_[idx].cost = new_node.cost + norm;
			graph_[idx].parent = new_node.idx;
			plotLine(new_node.pose, n.pose, {0.5, 0.5, 1});
		}
	}
}

void RRTStar::expandGraph(std::vector<double> x_r, int goal)
{
	// Find the closest node to x_r (x_goal or random point)
	node closest_node, possible_node;
	closest_node = closest(x_r);

	std::vector<double> p(x_r.size(), 0.0);
	double norm = euclideanDistance(x_r, closest_node.pose);

	double new_node_cost = 0.0;
	// Check if the closest node norm is less then 1. If so, return as closest node the random point or the goal.
	if (norm < epsilon_)
	{
		p = x_r;
		new_node_cost = norm;
	}
	else if (!goal)
	{
		// Else scale the vector to the epsilon distance
		for (uint i = 0; i < p.size(); i++)
		{
			p[i] = closest_node.pose[i] + (x_r[i] - closest_node.pose[i]) / norm * epsilon_;
		}
		new_node_cost = epsilon_;
	}
	else
	{
		p = obstacles_[0].center;
	}
	// Set the parent of the possible node to the closest node
	possible_node.pose = p;
	possible_node.parent = closest_node.idx;

	// Check if the possible node is in collision with the obstacles (Spheres, for now)
	if (!checkCollision(possible_node))
	{
		// Find the nearest set of nodes to the possible node
		std::vector<node> nearest_set = nearestSet(possible_node);
		// Set the cost of the possible node to the cost of the closest node + the distance between them
		possible_node.idx = graph_.back().idx + 1;
		possible_node.cost = closest_node.cost + new_node_cost;
		// Find the cheapest node in the nearest set from the possibil node to a node in the nearest set
		cheapest(possible_node, nearest_set);
		// Add the possible node to the graph
		graph_.push_back(possible_node);
		plotPoint(possible_node.pose);
		plotLine(possible_node.pose, closest_node.pose);
		// Rewire the tree in the neighborhood of the new node to minimize the cost.
		rewiring(possible_node, nearest_set);
	}
}

void RRTStar::checkEnd()
{
	int previous = graph_.size();
	// If expanding the graph with the goal point returns a node in the goal region, the algorithm is done.
	expandGraph(x_goal_, 1);
	if (graph_.size() > previous)
	{
		finish_ = true;
	}
}

void RRTStar::solve()
{
	// Cycle for it_max_ iterations
	while (graph_.size() < it_max_ && ros::ok())
	{
		std::vector<double> x_r(x_start_.size(), 0.0); // Tmp point
		it_++;										   // Increase the iteration counter
		// 50% to expand towards the goal or 50% to expand towards a random point
		if (it_ % 50 == 0)
		{
			x_r = x_goal_;
		}
		else
		{
			x_r = randomPoint();
		}
		expandGraph(x_r); // Expand the graph
		ros::Duration(0.01).sleep();
		ROS_INFO("Size graph %i", int(graph_.size()));
	}
	// After it_max_ iterations, check if the goal is reached
	checkEnd();

	if (finish_)
	{
		plotPath();
		ROS_INFO("Total cost %f", graph_.back().cost);
	}
	else
	{
		ROS_ERROR("Path not found!");
	}
	std::cout << "Press Entert to delete all markers...";
	char a;
	std::cin >> a;

	for (auto &m : markers_.markers)
	{
		m.action = 3; // Action 3 is to delete the marker
	}
	marker_pub_.publish(markers_);
	ros::shutdown();
}
void RRTStar::obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	visualization_msgs::Marker tmp;

	for (int i = 0; i < msg->markers.size(); i++)
	{
		if (msg->markers[i].id == 100 && check_obs_1 == false)
		{
			ROS_INFO("Obstacle 1 received!");
			tmp = msg->markers[i];
			obsList.push_back(tmp);
			check_obs_1 = true;
		}
		else if (msg->markers[i].id == 101 && check_obs_2 == false)
		{
			ROS_INFO("Obstacle 2 received!");
			tmp = msg->markers[i];
			obsList.push_back(tmp);
			check_obs_2 = true;
		}
	}
}
void RRTStar::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& data)
{
	//Set goal
	if (!goal_received)
	{
		goal.point.x = data->pose.position.x;
		goal.point.y = data->pose.position.y;
		goal.id = -1;

		goal_received = true;

		ROS_INFO("Goal received!");
		ROS_INFO("Goal x: %f", goal.point.x);
		ROS_INFO("Goal y: %f", goal.point.y);
	}
}
void RRTStar::a1PoseCallback(const nav_msgs::Odometry::ConstPtr& data)
{
	//Set initial position
	if (!init_received)
	{
		init.point.x = data->pose.pose.position.x;
		init.point.y = data->pose.pose.position.y;
		init.id = -2;
		init.parent_id = -3;

		tree.push_back(init);

		init_received = true;

		ROS_INFO("Initial position received!");
		ROS_INFO("Initial x: %f", init.point.x);
		ROS_INFO("Initial y: %f", init.point.y);
	}
}
void RRTStar::spinner()
{
	ros::spinOnce();
}
