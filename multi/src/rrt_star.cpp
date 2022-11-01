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
	// Max number of iterations.
	it_max_ = 500;
	
	// epsilon_ = distance between nodes
	epsilon_ = 1.0;

	// d_ = distance between nodes in the path
	d_ = 10.0;

	// Map limits

	int xmin = -10;
	int xmax = 10;
	int ymin = -10;
	int ymax = 10;
	int zmin = -10;
	int zmax = 10;

	// Map limits vector
	lim_ = {xmin, xmax, ymin, ymax, zmin, zmax};

	ROS_INFO("RRTStar node started");

	bringup();

	/*
	Points are vectors! The first element is the x coordinate, the second is the y coordinate,
	and the third is the z coordinate.
	*/

	// initialize the x_start_ = {x, y, z}; vector
	// x_start_ = {};

	// x_goal_ = {x, y, z};
	// x_goal_ = {};

	// n_ob = number of obstacles
	// int n_ob = 1;
	// double r_max = 1.0;

	// Random generator
	srand(time(NULL));

	// Obstacles vector clearup
	// obstacles_.clear();

	// Generate random SPHERE obstacles.
	// while (obstacles_.size() < n_ob && ros::ok())
	// {
	// 	sphere_obstacle new_ob;
	// 	new_ob.radius = (double)random() / RAND_MAX * r_max;
	// 	new_ob.center = randomPoint();
	// 	if (euclideanDistance(new_ob.center, x_start_) > new_ob.radius && euclideanDistance(new_ob.center, x_goal_) > new_ob.radius)
	// 	{
	// 		obstacles_.push_back(new_ob);
	// 	}
	// }

	initialize();
}

void RRTStar::bringup()
{
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	a1_pose_sub_ = nh_.subscribe("/a1_gazebo/a1/odom", 1, &RRTStar::a1PoseCallback, this);
	obs_sub_ = nh_.subscribe("visualization_marker_array", 1, &RRTStar::obstaclesCallback, this);
	goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &RRTStar::goalCallback, this);

	while (ros::ok)
	{
		ROS_INFO_ONCE("Waiting for goal, initial position and obstacles");
		ros::spinOnce();
		if (goal_received_ && init_received_ && obs_1_received_ && obs_2_received_)
		{
			break;
		}
	}

}

void RRTStar::initialize()
{
	ROS_INFO("RRTStar node initialized");
	// Publisher for the tree graph
	ros::Duration(1).sleep();

	// Add the start node to the tree
	plotPoint();

	// Plot the obstacles on the map
	// plotObstacles();

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
		mu_free_ -= (obstacles_[i].x_scale_ * obstacles_[i].y_scale_ * obstacles_[i].z_scale_) * 2;
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
	std::cout << "\nInsert maximum iteration number: " << std::endl;
	std::cin >> it_max_;

	// Solve the graph
	solve();
}

void RRTStar::plotPoint()
{  
	visualization_msgs::Marker start, end;
	geometry_msgs::Point p;

	start.type = end.type = visualization_msgs::Marker::POINTS;
	start.header.frame_id = end.header.frame_id = "map";
	start.header.stamp = end.header.stamp = ros::Time::now();
	start.ns = end.ns = "start/end";
	start.id = -10;
	end.id = -11;
	start.action = end.action = visualization_msgs::Marker::ADD;

	start.color.a = 1.0;
	start.color.g = 1.0;
	start.scale.x = start.scale.y = 0.1;
	end.scale.x = end.scale.y = 0.1;

	end.color.a = 1.0;
	end.color.r = 1.0;

	p.x = x_start_[0];
	p.y = x_start_[1];
	start.points.push_back(p);

	p.x = x_goal_[0];
	p.y = x_goal_[1];
	end.points.push_back(p);

	marker_pub_.publish(start);
	marker_pub_.publish(end);  
}

// void RRTStar::plotObstacles()
// {
// 	for (auto &x : obstacles_)
// 		plotPoint(x.center, {0, 0.75, 0.75}, x.radius / 0.01);
// }

void RRTStar::plotLine(std::vector<double> x1, std::vector<double> x2, std::vector<double> c, double w)
{
	marker_.type = visualization_msgs::Marker::LINE_LIST;
	marker_.header.frame_id = "map";
	marker_.header.stamp = ros::Time::now();
	marker_.ns = "RRTStar";
	marker_.id = markers_.markers.size() + 1;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.scale.x = 0.02;
	marker_.color.a = 0.5;
	marker_.color.r = c[0];
	marker_.color.g = c[1];
	marker_.color.b = c[2];

	geometry_msgs::Point x;
	x.z = 0.0;

	x.x = x1[0];
	x.y = x1[1];
	marker_.points.push_back(x);

	x.x = x2[0];
	x.y = x2[1];
	marker_.points.push_back(x);

	markers_.markers.push_back(marker_);
	marker_pub_.publish(marker_);
}

void RRTStar::generate_path()
{
	// Plot the final path if it exists
	geometry_msgs::PoseStamped pose;

	pose.pose.position.z = 0;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 1;

	int idx = graph_.size() - 1;

	double total_cost = 0.;

	node tmp = graph_[idx];

	while (tmp.idx != -1)
	{
		pose.pose.position.x = tmp.pose[0];
		pose.pose.position.y = tmp.pose[1];
		path_.poses.push_back(pose);
		total_cost += tmp.cost;
		plotLine(tmp.pose, graph_[tmp.parent].pose, {0.5, 0.5, 0.5}, 0.1);
		tmp = graph_[tmp.parent];
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

bool RRTStar::intersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
	// Check for line intersection
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	
	float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
	float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

	// if uA and uB are between 0-1, lines are colliding
	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool RRTStar::checkRectangleIntersection(std::vector<double> parent, std::vector<double> possible)
{
	// x1 = Parent Node pose
	// x2 = Possible Node pose

	float x1_ = parent[0];
	float y1_ = parent[1];
	float x2_ = possible[0];
	float y2_ = possible[1];
	
	for (int i = 0; i < obstacles_.size(); ++i)
	{
		// Inflated Obstacle
		float obs_xl = (obstacles_[i].center[0] - obstacles_[i].x_scale_ / 2) - 0.3; // X Left
		float obs_xr = (obstacles_[i].center[0] + obstacles_[i].x_scale_ / 2) + 0.3; // X Right
		float obs_yb = (obstacles_[i].center[1] - obstacles_[i].y_scale_ / 2) - 0.3; // Y Bottom
		float obs_yt = (obstacles_[i].center[1] + obstacles_[i].y_scale_ / 2) + 0.3; // Y Top
		
		bool bottom = intersect(x1_, y1_, x2_, y2_, obs_xl, obs_yb, obs_xr, obs_yb);

		bool left = intersect(x1_, y1_, x2_, y2_, obs_xl, obs_yb, obs_xl, obs_yt);

		bool right = intersect(x1_, y1_, x2_, y2_, obs_xr, obs_yb, obs_xr, obs_yt);

		bool top = intersect(x1_, y1_, x2_, y2_, obs_xl, obs_yt, obs_xr, obs_yt);

		// Collision Check
		if (bottom || left || right || top)
		{
			ROS_INFO("Collision Detected");
			return false;
		}
	}
	// World Boundary Check
	if (x2_ < lim_[1] || x2_ > lim_[0] || y2_ < lim_[3] || y2_ > lim_[2])
	{
		return true;
	}
	else
	{
		ROS_INFO("Collision Detected");
		return false;
	}
}

bool RRTStar::feasible(node possible_node)
{
	// Check if the new node is in collision with the obstacles
	node parent = graph_[possible_node.parent];
	return checkRectangleIntersection(parent.pose, possible_node.pose);
}

bool RRTStar::feasible(node node1, node node2)
{
	// Check if the new node is in collision with the obstacles

	return checkRectangleIntersection(node2.pose, node1.pose);
}

void RRTStar::cheapest(node &new_node, std::vector<node> nearest_set)
{
	// Find the cheapest node in the nearest set
	for (auto &n : nearest_set)
	{
		double norm = euclideanDistance(new_node.pose, n.pose);
		if ((n.cost + norm) < new_node.cost && feasible(new_node, n))
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
		if ((new_node.cost + norm) < n.cost && feasible(new_node, n))
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

	ROS_INFO("Closest Node: %f, %f", closest_node.pose[0], closest_node.pose[1]);

	std::vector<double> p(x_r.size(), 0.0);
	double norm = euclideanDistance(x_r, closest_node.pose);

	double new_node_cost = 0.0;
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

	ROS_INFO("Possible Node: %f, %f", possible_node.pose[0], possible_node.pose[1]);

	// Check if the possible node is in collision with the obstacles (Spheres, for now)
	if (feasible(possible_node))
	{
		ROS_INFO("Node feasible");
		// Find the nearest set of nodes to the possible node
		std::vector<node> nearest_set = nearestSet(possible_node);
		// Set the cost of the possible node to the cost of the closest node + the distance between them
		possible_node.idx = graph_.back().idx + 1;
		possible_node.cost = closest_node.cost + new_node_cost;
		// Find the cheapest node in the nearest set from the possibil node to a node in the nearest set
		cheapest(possible_node, nearest_set);
		// Add the possible node to the graph
		graph_.push_back(possible_node);
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
	while (ros::ok)
	{

	}
	// After it_max_ iterations, check if the goal is reached
	checkEnd();

	if (finish_)
	{
		generate_path();
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
	for (int i = 0; i < msg->markers.size(); i++)
	{
		if (msg->markers[i].id == -100 && obs_1_received_ == false)
		{
			ROS_INFO("Obstacle 1 received: ");
			ROS_INFO("Center: %f, %f", msg->markers[i].pose.position.x, msg->markers[i].pose.position.y);
			ROS_INFO("Scale x: %f", msg->markers[i].scale.x);
			ROS_INFO("Scale y: %f", msg->markers[i].scale.y);
			rectangle_obstacle obs;
			obs.center = {msg->markers[i].pose.position.x, msg->markers[i].pose.position.y};
			obs.x_scale_ = msg->markers[i].scale.x;
			obs.y_scale_ = msg->markers[i].scale.y;
			obs.z_scale_ = msg->markers[i].scale.z;
			obstacles_.push_back(obs);
			obs_1_received_ = true;
		}
		else if (msg->markers[i].id == -101 && obs_2_received_ == false)
		{
			ROS_INFO("Obstacle 2 received!");
			ROS_INFO("Center: %f, %f", msg->markers[i].pose.position.x, msg->markers[i].pose.position.y);
			ROS_INFO("Scale x: %f", msg->markers[i].scale.x);
			ROS_INFO("Scale y: %f", msg->markers[i].scale.y);
			rectangle_obstacle obs;
			obs.center = {msg->markers[i].pose.position.x, msg->markers[i].pose.position.y};
			obs.x_scale_ = msg->markers[i].scale.x;
			obs.y_scale_ = msg->markers[i].scale.y;
			obs.z_scale_ = msg->markers[i].scale.z;
			obstacles_.push_back(obs);
			obs_2_received_ = true;
		}
	}
}

bool RRTStar::goal_feasible() 
{
	for (int i = 0; i < obstacles_.size(); i++)
	{
		if (x_goal_[0] > obstacles_[i].center[0] - obstacles_[i].x_scale_/2 
			&& x_goal_[0] < obstacles_[i].center[0] + obstacles_[i].x_scale_/2 
			&& x_goal_[1] > obstacles_[i].center[1] - obstacles_[i].y_scale_/2 
			&& x_goal_[1] < obstacles_[i].center[1] + obstacles_[i].y_scale_/2)
		{
			return false;
		}
	}
	return true;
}

void RRTStar::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& data)
{

	//Set goal
	if (!goal_received_)
	{
		x_goal_ = {data->pose.position.x, data->pose.position.y};

		if (x_goal_[0] > lim_[1] || x_goal_[0] < lim_[0] || x_goal_[1] > lim_[3] || x_goal_[1] < lim_[2])
		{
			ROS_ERROR("Goal out of bounds! Retry...");
			x_goal_.clear();
		}
		else if (x_goal_[0] == x_start_[0] && x_goal_[1] == x_start_[1])
		{
			ROS_ERROR("Goal and start are the same! Retry...");
			x_goal_.clear();
		}
		else if (!goal_feasible())
		{
			ROS_ERROR("Goal is inside an obstacle! Retry...");
			x_goal_.clear();
		}
		else
		{
			ROS_INFO("Goal received!");
			goal_received_ = true;
		}
	}
}
void RRTStar::a1PoseCallback(const nav_msgs::Odometry::ConstPtr& data)
{
	//Set initial position
	if (!init_received_)
	{
		x_start_ = {data->pose.pose.position.x, data->pose.pose.position.y};

		init_received_ = true;

		ROS_INFO("Initial position received!");
	}
}
void RRTStar::spinner()
{
	ros::spinOnce();
}
