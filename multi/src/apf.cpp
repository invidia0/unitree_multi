#include "APF/APF.h"

APF::APF()
{
	int ka = 1;

	// Map limits
	int xmin = -10;
	int xmax = 10;
	int ymin = -10;
	int ymax = 10;
	int zmin = -10;
	int zmax = 10;

	// Map limits vector
	lim_ = {xmin, xmax, ymin, ymax, zmin, zmax};


    // obs_sub_ = nh_.subscribe("visualization_marker_array", 1, &APF::obstaclesCallback, this);
	bringup();

	ROS_INFO("Goal: %f, %f", x_goal_[0], x_goal_[1]);
	// a1_pose_sub_ = nh_.subscribe("/a1_gazebo/a1/odom", 1, &APF::a1PoseCallback, this);
	
    for (int i = xmin; i < xmax; i++)
	{
		for (int j = ymin; j < ymax; j++)
		{
			x_curr_ = {(double)i, (double)j};
			distance_goal_ = euclideanDistance(x_curr_, x_goal_);	
			gradient_goal_ = ka * distance_goal_;

			markerDeclaration(x_curr_, x_goal_);
		}
	}

	while (ros::ok())
	{
		marker_pub_.publish(markers_);
		ros::Duration(0.1).sleep();
	}
}
void APF::bringup()
{
	marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

	goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &APF::goalCallback, this);
	a1_pose_sub_ = nh_.subscribe("/a1_gazebo/a1/odom", 1, &APF::a1PoseCallback, this);

	while (ros::ok)
	{
		ROS_INFO_ONCE("Waiting for goal");
		ros::spinOnce();
		if (goal_received_)
		{
			visualization_msgs::Marker goal;
			goal.header.frame_id = "map";
			goal.header.stamp = ros::Time::now();
			goal.ns = "goal";
			goal.id = 0;
			goal.type = visualization_msgs::Marker::SPHERE;

			goal.action = visualization_msgs::Marker::ADD;
			goal.pose.position.x = x_goal_[0];
			goal.pose.position.y = x_goal_[1];
			goal.pose.position.z = 0.0;
			goal.pose.orientation.x = 0.0;
			goal.pose.orientation.y = 0.0;
			goal.pose.orientation.z = 0.0;
			goal.pose.orientation.w = 1.0;

			goal.scale.x = 0.2;
			goal.scale.y = 0.2;
			goal.scale.z = 0.2;
			
			goal.color.a = 1.0;
			goal.color.r = 1.0;
			goal.color.g = 1.0;
			goal.color.b = 0.0;

			markers_.markers.push_back(goal);
			marker_pub_.publish(markers_);
			break;
		}
	}
}
void APF::markerDeclaration(std::vector<double> x1, std::vector<double> x2)
{
	// x1 = CURR
	// x2 = GOAL
	visualization_msgs::Marker marker_;
	// Set the marker type ARROW=0
    marker_.type = 0;
    marker_.header.frame_id = "map";
    marker_.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = "gradient_field";
    marker_.id = markers_.markers.size();
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_.action = visualization_msgs::Marker::ADD;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_.color.r = 0.0;
    marker_.color.g = 0.0;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker_.scale.x = 0.1;
	marker_.scale.y = 0.1;
	marker_.scale.z = 0.2;

	geometry_msgs::Point p;

	p.x = x1[0];
	p.y = x1[1];

	marker_.points.push_back(p);

	if (x2[0] > x1[0])
	{
		p.x = x1[0] + (x2[0] - x1[0]) / distance_goal_ * 0.2;
	}
	else
	{
		p.x = x1[0] - (x1[0] - x2[0]) / distance_goal_ * 0.2;
	}

	if (x2[1] > x1[1])
	{
		p.y = x1[1] + (x2[1] - x1[1]) / distance_goal_ * 0.2;
	}
	else
	{
		p.y = x1[1] - (x1[1] - x2[1]) / distance_goal_ * 0.2;
	}

	marker_.points.push_back(p);
	markers_.markers.push_back(marker_);
	marker_pub_.publish(markers_);
}
void APF::obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
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
void APF::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& data)
{
	ROS_INFO("GOAL CALLBACK");
	//Set goal
	if (!goal_received_)
	{
		x_goal_ = {data->pose.position.x, data->pose.position.y};

		ROS_INFO("Check goal");

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
		// else if (!goal_feasible())
		// {
		// 	ROS_ERROR("Goal is inside an obstacle! Retry...");
		// 	x_goal_.clear();
		// }
		else
		{
			std::cout << ">> Goal received!" << std::endl;
			goal_received_ = true;
		}
	}
}

double APF::euclideanDistance(std::vector<double> x1, std::vector<double> x2)
{
	double quad_dist = 0.0;
	for (uint i = 0; i < x1.size(); i++)
	{
		quad_dist += std::pow(x1[i] - x2[i], 2);
	}
	return std::sqrt(quad_dist);
}

void APF::a1PoseCallback(const nav_msgs::Odometry::ConstPtr& data)
{
	//Set initial position
	if (!init_received_)
	{
		x_start_ = {data->pose.pose.position.x, data->pose.pose.position.y};

		init_received_ = true;

		std::cout << ">> Initial position received!" << std::endl;
	}
}

void APF::spinner()
{
	ros::spinOnce();
}