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

	std::cout << "================================" << std::endl;
	std::cout << "===> RRT* algorithm started <===" << std::endl;
	std::cout << "================================" << std::endl;

	bringup();

	/*
	Points are vectors! The first element is the x coordinate, the second is the y coordinate,
	and the third is the z coordinate.
	*/

	// Random generator
	srand(time(NULL));

	std::cout << "================================" << std::endl;
	initialize();
}

void RRTStar::bringup()
{
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);

	a1_pose_sub_ = nh_.subscribe("/a1_gazebo/a1/odom", 1, &RRTStar::a1PoseCallback, this);
	obs_sub_ = nh_.subscribe("visualization_marker_array", 1, &RRTStar::obstaclesCallback, this);
	goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &RRTStar::goalCallback, this);

	std::cout << ">> Subscribers and publishers initialized" << std::endl;
	std::cout << ">> Waiting for the robot pose, goal and obstacles..." << std::endl;
	while (ros::ok)
	{
		ros::spinOnce();
		if (goal_received_ && init_received_ && obs_1_received_ && obs_2_received_)
		{
			break;
		}
	}

}

void RRTStar::initialize()
{
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
	std::cout << ">> Insert maximum iteration number: ";
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

void RRTStar::plotLine(std::vector<double> x1, std::vector<double> x2, std::vector<double> c, double i)
{
	marker_.type = visualization_msgs::Marker::LINE_LIST;
	marker_.header.frame_id = "map";
	marker_.header.stamp = ros::Time::now();
	marker_.ns = "RRTStar";
	marker_.id = markers_.markers.size() + i;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.scale.x = 0.02;
	marker_.color.a = 0.2;
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
	int idx = graph_.size() - 1;

	double total_cost = 0.;

	node tmp = graph_[idx];
	std::cout << ">> Path found! Nodes:" << std::endl;
	while (tmp.idx != -1)
	{
		std::cout << "Node " << tmp.idx << " with cost " << tmp.cost << std::endl;
		path_.push_back(tmp);
		total_cost += tmp.cost;
		if (tmp.parent == -1)
			break;
		plotLine(tmp.pose, graph_[tmp.parent].pose, {0.5, 0.5, 0.5}, 0.1);
		tmp = graph_[tmp.parent];
	}
	std::cout << ">> Total cost: " << total_cost << std::endl;
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

	// Check if the possible node is in collision with the obstacles (Spheres, for now)
	if (feasible(possible_node))
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

void RRTStar::bezier_curve_generator()
{
	/*
	Reference:
	- https://en.wikipedia.org/wiki/B%C3%A9zier_curve
	- https://www.geeksforgeeks.org/cubic-bezier-curve-implementation-in-c/
	- https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm

	Bezier curve form:
	P(u) = sum(i=0,3) Bi,n(u) * Pi
	Where:
	Bi,n(u) = Bernstein polynomial of degree n and index i and u is the variable between 0 and 1
	So a bezier curve is defined by a set of control points P0 to Pn
	where n is called its order(n = 1 for linear, n = 2 for quadratic, n = 3 for cubic)
	Cubic Bezier curve is defined by 4 control points P0 to P3:

	P(u) = (1-u)^3 * P0 + 3u(1-u)^2 * P1 + 3u^2(1-u) * P2 + u^3 * P3

	B0(u) = (1-u)^3

	B1(u) = (3u^3 - 6u^2 + 4)

	B2(u) = (-3u^3 + 3u^2 + 3u + 1)

	B3(u) = u^3


	So:

	x(u) = (1-u)^3 * x0 + 3u(1-u)^2 * x1 + 3u^2(1-u) * x2 + u^3 * x3
	y(u) = (1-u)^3 * y0 + 3u(1-u)^2 * y1 + 3u^2(1-u) * y2 + u^3 * y3

	We then divide eaxh Bernstein polynomial by 6. This is because the sum of the Bernstein polynomials is 6. 
	*/

	std::vector<double> x;
	std::vector<double> y;

	float Px, Py;
	float B0, B1, B2, B3;

	int steps = 10;
	int n_points = path_.size();

	for (int i = 0; i < path_.size(); ++i)
	{
		x.push_back(path_[i].pose[0]);
		y.push_back(path_[i].pose[1]);
	}

	for (int i = 0; i < n_points - 3; i++)
	{
		for (int j = 0; j <= steps; j++)
		{
			float u = float(j) / float(steps);

			//Calculate Bernstein polynomials
			B0 = float(pow(1-u, 3)/6);

			B1 = float((3 * pow(u, 3) - 6 * pow(u, 2) + 4)/6);

			B2 = float((-3 * pow(u, 3) + 3 * pow(u, 2) + 3 * u + 1)/6);

			B3 = float(pow(u, 3)/6);

			//Calculate x and y coordinates
			Px = B0 * x[i] + B1 * x[i + 1] + B2 * x[i + 2] + B3 * x[i + 3];

			Py = B0 * y[i] + B1 * y[i + 1] + B2 * y[i + 2] + B3 * y[i + 3];

			geometry_msgs::Point tmp_point;
			tmp_point.x = Px;
			tmp_point.y = Py;
			tmp_point.z = 0;

			smoothed_path_.push_back(tmp_point);
		}
	}
}

void RRTStar::send_path()
{
	nav_msgs::Path path_msg;
	geometry_msgs::PoseStamped pose;

	pose.pose.position.z = 0;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 1;

	for (int i = 0; i < smoothed_path_.size(); i++)
	{
		pose.pose.position.x = smoothed_path_[i].x;
		pose.pose.position.y = smoothed_path_[i].y;

		path_msg.poses.push_back(pose);
	}

	path_pub_.publish(path_msg);
	std::cout << "=========================" << std::endl;
	std::cout << "===> Path published! <===" << std::endl;
	std::cout << "=========================" << std::endl;
}

void RRTStar::add_final_path(geometry_msgs::Point curr, geometry_msgs::Point next)
{
	final_edge_.type = visualization_msgs::Marker::LINE_LIST;
	final_edge_.header.frame_id = "map";
	final_edge_.header.stamp = ros::Time::now();
	final_edge_.ns = "final_path";
	final_edge_.id = -4;
	final_edge_.action = visualization_msgs::Marker::ADD;
	final_edge_.pose.orientation.w = 1;

	final_edge_.scale.x = 0.04;

	final_edge_.color.g = final_edge_.color.r = 1;
	final_edge_.color.a = 1.0;

	final_edge_.points.push_back(curr);
	final_edge_.points.push_back(next);

	marker_pub_.publish(final_edge_);
}

void RRTStar::solve()
{
	auto t1 = std::chrono::high_resolution_clock::now();
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
		std::cout << ">> Iteration: " << int(graph_.size()) << std::endl;
	}

	// After it_max_ iterations, check if the goal is reached
	checkEnd();

	if (finish_)
	{
		geometry_msgs::Pose tmp_pose;
		generate_path();

		std::reverse(path_.begin(), path_.end());

		for (int i = 0; i < 2; i++)
		{
			path_.insert(path_.begin(), path_[0]);
		}

		for (int i = 0; i < 3; i++)
		{
			path_.push_back(path_.back());
		}

		std::cout << ">> Smoothing path..." << std::endl;
		bezier_curve_generator();

		geometry_msgs::Point tmp_curr;
		geometry_msgs::Point tmp_next;

		for (int i = 0; i < smoothed_path_.size() - 1; i++)
		{
			tmp_curr = smoothed_path_[i];
			tmp_next = smoothed_path_[i + 1];

			add_final_path(tmp_curr, tmp_next);
			//plotLine({tmp_curr.x, tmp_curr.y}, {tmp_next.x, tmp_next.y}, {1, 1, 0}, 5);
		}

		send_path();
	}
	else
	{
		ROS_WARN("Path not found!");
	}
	auto t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	std::cout << ">> Total time taken: " << ms_double.count() << " ms" << std::endl;
}
void RRTStar::obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	for (int i = 0; i < msg->markers.size(); i++)
	{
		if (msg->markers[i].id == -100 && obs_1_received_ == false)
		{
			std::cout << ">> Obstacle 1 received!" << std::endl;
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
			std::cout << ">> Obstacle 2 received!" << std::endl;
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
			std::cout << ">> Goal received!" << std::endl;
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

		std::cout << ">> Initial position received!" << std::endl;
	}
}
void RRTStar::spinner()
{
	ros::spinOnce();
}
