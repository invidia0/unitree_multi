#include "APF/APF.h"

PotentialField::PotentialField()
{
	int ka	=	1;
	int kr	=	1;
	int Q	=	1;
	// Map limits
	int xmin = -10;
	int xmax = 10;
	int ymin = -10;
	int ymax = 10;
	int zmin = -10;
	int zmax = 10;

	// Map limits vector
	lim_ = {xmin, xmax, ymin, ymax, zmin, zmax};


    obs_sub_ = nh_.subscribe("visualization_marker_array", 1, &APF::obstaclesCallback, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &APF::goalCallback, this);
	a1_pose_sub_ = nh_.subscribe("/a1_gazebo/a1/odom", 1, &APF::a1PoseCallback, this);

	double gradient_goal;
	double gradient_obs[obs_sub_.size()];
    double distance_goal;
	
    for(int i=xmin;	i<xmax; i++)
	{
		double pose[2];
		pose.pushback(i);
		for(int j=ymin;	j<ymax; j++)
		{
			pose.pushback(j);
			distance_goal = euclideanDistance(pose, goal_sub_);	
			gradient_goal=ka*distance_goal;

			std::vector<visualization_msgs>::Marker marker_;		
			// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
			marker_.pose.position.x	= i;
			marker_.pose.position.y = j;
			marker_.pose.position.z = 1;
			marker_.pose.orientation.x = goal_sub.pose.x-pose[0];
			marker_.pose.orientation.y = goal_sub.pose.y-pose[1];
			marker_.pose.orientation.z = 0.0;
			marker_.pose.orientation.w = 1.0;

			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker_.scale.x = gradient_goal;
			marker_.scale.y = 1.0;
			marker_.scale.z = 1.0;

			marker.pushback(marker_);
		}
	}
	for (int k = 0; k < obs_sub_.size(); k++)
	{
		for(int i=xmin;	i<xmax; i++)
		{
			double pose[2];
			pose.pushback(i);
			for(int j=ymin;	j<ymax; j++)
			{
				pose.pushback(j);
				distance_goal = euclideanDistance(pose, obs_sub_);
				if(distance_goal < Q)
					gradient_obs.pushback(kr*(1/Q-1/distance_goal)*1/(distance_goal*distance_goal));
				else
					gradient_obs.pushback(0);

				std::vector<visualization_msgs>::Marker marker_obs_;		
				// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
				marker_.pose.position.x	= i;
				marker_.pose.position.y = j;
				marker_.pose.position.z = 1;
				marker_.pose.orientation.x = gradient_obs[obs_sub_.size()].pose.x-pose[0];
				marker_.pose.orientation.y = gradient_obs[obs_sub_.size()].pose.y-pose[1];
				marker_.pose.orientation.z = 0.0;
				marker_.pose.orientation.w = 1.0;
				marker.color.a = 1.0; 
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				// Set the scale of the marker -- 1x1x1 here means 1m on a side
				marker_.scale.x = gradient_obs[obs_sub_.size()];
				marker_.scale.y = 1.0;
				marker_.scale.z = 1.0;

				marker.pushback(marker_obs_);	
			
			}
		}
	}
}
void APF::markerDeclaration()
{
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	std::vector<visualization_msgs>::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type ARROW=0
    marker.type = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
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

double APF::euclideanDistance(std::vector<double> x1, std::vector<double> x2)
{
	double quad_dist = 0.0;
	for (uint i = 0; i < x1.size(); i++)
	{
		quad_dist += std::pow(x1[i] - x2[i], 2);
	}
	return std::sqrt(quad_dist);
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