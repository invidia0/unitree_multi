#include <random>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <ros/ros.h>

class Planner
{
private:
    struct Node
    {
        geometry_msgs::Point point;
        Node(geometry_msgs::Point p) : point(p) {}

        int id;
        int parent_id;

        Node () {}
    };
    
    //Ros initialization
    ros::NodeHandle nh;
    ros::Publisher marker_pub, path_pub;
    ros::Subscriber goal_sub, a1_pose, obs_sub;

    //Vector initialization
    std::vector<Node> tree;
    std::vector<Node> path;
    std::vector<Node> parentList;
    std::vector<geometry_msgs::Point> smoothed_path;
    std::vector<visualization_msgs::Marker> obsList;
    std::map<float, Node> distance_map;

    //Msgs
    geometry_msgs::Pose pose;
    nav_msgs::Path path_msg;

    //Visualization Marker
    visualization_msgs::Marker edge, final_edge;

    //Node initialization
    Node init, goal;

    //Map definition
    int x_max = 10;
    int x_min = -10;
    int y_max = 10;
    int y_min = -10;
    
    //Variables
    int frame_id = 0;
    float sigma = 0.9;
    bool goal_received = false;
    bool init_received = false;
    bool obs_received = false;
    bool check_obs_1 = false;
    bool check_obs_2 = false;
    bool path_found = false;
    bool path_published = false;
    bool no_collision = false;

public:
    Planner()
    {
        //Vectors initialization
        tree.reserve(1000);
        path.reserve(100);
        obsList.reserve(2);

        start_up();

        rviz_bringup();

        while (ros::ok)
        {
            ROS_INFO_ONCE("RRT Running!");

            if (!path_found)
            {
                Node new_node = generate_node();

                geometry_msgs::Point new_pos = new_node.point;

                float distance = sqrt(pow(new_node.point.x - goal.point.x, 2) +
                                      pow(new_node.point.y - goal.point.y, 2));

                if (distance <= 1)
                {
                    add_edge(new_node.point, goal.point);

                    goal.parent_id = new_node.id;

                    path_found = true;
                }

            }
            else if (path_found && !path_published)
            {
                ROS_INFO("Path Found!");

                path.push_back(goal);

                int tmp_parent_id = goal.parent_id;

                while (tmp_parent_id != init.parent_id)
                {
                    for (int i =  tree.size() - 1; i >= 0; i--)
                    {
                        if (tree[i].id == tmp_parent_id)
                        {
                            path.push_back(tree[i]);
                            
                            tmp_parent_id = tree[i].parent_id;
                        }
                    }
                }

                std::reverse(path.begin(), path.end());

                for (int i = 0; i < 2; i++)
                {
                    path.insert(path.begin(), init);
                }
                for (int i = 0; i < 3; i++)
                {
                    path.push_back(goal);
                }

                bezier_curve_generator();

                geometry_msgs::Point tmp_curr;
                geometry_msgs::Point tmp_next;

                for (int i = 0; i < smoothed_path.size() - 1; i++)
                {
                    tmp_curr = smoothed_path[i];
                    tmp_next = smoothed_path[i + 1];

                    add_final_path(tmp_curr, tmp_next);
                }

                send_path();

                for (int i = 0; i < 10; i++)
                {
                    std::cout << "Smoothed Path: " << smoothed_path[i].x << " " << smoothed_path[i].y << std::endl;
                }
                
                path_published = true;
            }
            else
            {
                marker_pub.publish(edge);
                marker_pub.publish(final_edge);
            }

            while (marker_pub.getNumSubscribers() < 1)
            {
                ROS_WARN_ONCE("RVIZ Offline!");
                sleep(1);
            }         

            ros::Rate loop_rate(5);
            loop_rate.sleep();
            frame_id++;
        }
    }
    void bezier_curve_generator() {
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
        int n_points = path.size();

        for (int i = 0; i < path.size(); ++i)
        {
            x.push_back(path[i].point.x);
            y.push_back(path[i].point.y);
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

                smoothed_path.push_back(tmp_point);
            }
        }
    }
    void send_path()
    {
        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped pose;

        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        for (int i = 0; i < smoothed_path.size(); i++)
        {
            pose.pose.position.x = smoothed_path[i].x;
            pose.pose.position.y = smoothed_path[i].y;

            path_msg.poses.push_back(pose);
        }

        path_pub.publish(path_msg);
        path_published = true;

        ROS_INFO("Path Published!");
    }
    void add_final_path(geometry_msgs::Point curr, geometry_msgs::Point next)
    {
        final_edge.type = visualization_msgs::Marker::LINE_LIST;
        final_edge.header.frame_id = "map";
        final_edge.header.stamp = ros::Time::now();
        final_edge.ns = "final_path";
        final_edge.id = 4;
        final_edge.action = visualization_msgs::Marker::ADD;
        final_edge.pose.orientation.w = 1;

        final_edge.scale.x = 0.04;

        final_edge.color.g = final_edge.color.r = 1;
        final_edge.color.a = 1.0;

        final_edge.points.push_back(curr);
        final_edge.points.push_back(next);

        marker_pub.publish(final_edge);
    }
    Node generate_node()
    {
        geometry_msgs::Point random_point = generate_random_point();
        geometry_msgs::Point tmp;

        Node near_node = find_nearest_node(random_point);
        Node random_node(random_point);
        Node new_node;

        //Flip a coin to decide whether to expand towards the random point or the goal
        double coin = rand() / double(RAND_MAX);

        if (coin > 0.5)
        {
            new_node = extend(near_node, random_node);

        }
        else
        {
            new_node = extend(near_node, goal);
        }

        if(new_node.point.x != near_node.point.x || new_node.point.y != near_node.point.y )
        {   
            ROS_INFO("New node added!");
            add_edge(near_node.point, new_node.point);
        }

        return new_node;
    }
    void add_edge(geometry_msgs::Point near_node, geometry_msgs::Point next_node)
    {
        edge.type = visualization_msgs::Marker::LINE_LIST;
        edge.header.frame_id = "map";
        edge.header.stamp = ros::Time::now();
        edge.ns = "edges";
        edge.id = 2;
        edge.action = visualization_msgs::Marker::ADD;
        edge.pose.orientation.w = 1;

        edge.scale.x = 0.02;

        edge.color.b = 1.0;

        edge.color.a = 0.5;

        edge.points.push_back(near_node);
        edge.points.push_back(next_node);

        marker_pub.publish(edge);     
    }
    Node extend(Node near_node, Node random_node)
    {
        double slope;

        if (near_node.point.x != random_node.point.x)
        {
            slope = (random_node.point.y - near_node.point.y) / (random_node.point.x - near_node.point.x);
        }

        float theta = atan(slope);

        if (theta < 0)
        {
            if (random_node.point.x < near_node.point.x)
            {
                theta = theta + M_PI;
            }
            else
            {
                theta = theta + 2 * M_PI;
            }
        }
        else
        {
            if (((random_node.point.y - near_node.point.y) < 0) && ((random_node.point.x - near_node.point.x) < 0))
            {
                theta = theta + M_PI;
            }
        }

        float sin_theta = sin(theta);
        float cos_theta = cos(theta);

        Node next_node;
        next_node.point.x = near_node.point.x + sigma * cos_theta;
        next_node.point.y = near_node.point.y + sigma * sin_theta;
        next_node.id = frame_id;

        if (check_collision(near_node.point, next_node.point))
        {
            //no_collision = true;

            std::vector<Node>::iterator walk = parentList.begin();

            walk = parentList.insert(walk, near_node);
            next_node.parent_id = near_node.id;

            tree.push_back(next_node);
            return next_node;
        }
        else
        {
            return near_node;
        }
    }
    bool check_collision(geometry_msgs::Point near_point, geometry_msgs::Point next_point)
    {

        float x1 = near_point.x;
        float y1 = near_point.y;
        float x2 = next_point.x;
        float y2 = next_point.y;

        for (int i = 0; i < obsList.size(); i++)
        {
            visualization_msgs::Marker obs = obsList[i];

            float obs_xl = (obs.pose.position.x - obs.scale.x / 2) - 0.5; // X Left
            float obs_xr = (obs.pose.position.x + obs.scale.x / 2) + 0.5; // X Right
            float obs_yb = (obs.pose.position.y - obs.scale.y / 2) - 0.5; // Y Bottom
            float obs_yt = (obs.pose.position.y + obs.scale.y / 2) + 0.5; // Y Top
            /*
                        Y Top
                    ---------------
                    |             |
                    |             |
                    |             |
            X Left  |             |  X Right
                    |             |
                    |             |
                    |             |
                    ---------------
                        Y Bottom
            */

            bool bottom = intersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xr, obs_yb);

            bool left = intersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xl, obs_yt);

            bool right = intersect(x1, y1, x2, y2, obs_xr, obs_yb, obs_xr, obs_yt);

            bool top = intersect(x1, y1, x2, y2, obs_xl, obs_yt, obs_xr, obs_yt);

            if (bottom || left || right || top)
            {
                return false;
            }
        }
        //Check if the new node is inside the world
        if (x2 < x_max || x2 > x_min || y2 < y_max || y2 > y_min)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool intersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
    {
        // Check for line intersection
        // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
        // 2 points for each line
        float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

        // if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1)
        {

            float x_intersect = x1 + (uA * (x2 - x1));
            float y_intersect = y1 + (uA * (y2 - y1));

            return true;
        }
        else
        {
            return false;
        }
    }
    Node find_nearest_node(geometry_msgs::Point random_point)
    {
        //Clear the map
        distance_map.clear();

        //First cycle = only init node
        if (tree.size() == 1)
        {
            return tree[0];
        }

        //Find the nearest node
        for (int i = 0; i < tree.size(); i++)
        {
            float distance = std::sqrt(std::pow(random_point.x - tree[i].point.x, 2) + 
                                       std::pow(random_point.y - tree[i].point.y, 2));
            
            distance_map[distance] = tree[i];
        }

        //Maps are sorted by key, so the first element is the nearest node
        return distance_map.begin()->second;

    }
    geometry_msgs::Point generate_random_point()
    {
        //Unbiased random number generator
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<int> distribution_x(x_min, x_max);
        std::uniform_int_distribution<int> distribution_y(y_min, y_max);

        geometry_msgs::Point p;
        p.x = distribution_x(generator);
        p.y = distribution_y(generator);

        return p;
    }
    void rviz_bringup()
    {
        ROS_INFO("Starting rviz");

        //Visualize the initial node and goal node
        visualization_msgs::Marker start, end;
        geometry_msgs::Point p;

        start.type = end.type = visualization_msgs::Marker::POINTS;
        start.header.frame_id = end.header.frame_id = "map";
        start.header.stamp = end.header.stamp = ros::Time::now();
        start.ns = end.ns = "start/end";
        start.id = 0;
        end.id = 1;
        start.action = end.action = visualization_msgs::Marker::ADD;

        start.color.a = 1.0;
        start.color.g = 1.0;
        start.scale.x = start.scale.y = 0.1;
        end.scale.x = end.scale.y = 0.1;

        end.color.a = 1.0;
        end.color.r = 1.0;

        p.x = init.point.x;
        p.y = init.point.y;
        start.points.push_back(p);

        p.x = goal.point.x;
        p.y = goal.point.y;
        end.points.push_back(p);

        marker_pub.publish(start);
        marker_pub.publish(end);     
    }
    void start_up()
    {
        //Publishers
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        path_pub = nh.advertise<nav_msgs::Path>("path", 1);
        //Subscribers
        goal_sub = nh.subscribe("move_base_simple/goal", 1, &Planner::goal_callback, this);
        a1_pose = nh.subscribe("/a1_gazebo/a1/odom", 1, &Planner::a1_pose_callback, this);
        obs_sub = nh.subscribe("visualization_marker_array", 1, &Planner::obs_callback, this);

        //Waiting for everything to be received
        while (ros::ok)
        {
            ros::spinOnce();

            if (goal_received && init_received && check_obs_1 && check_obs_2)
            {
                ROS_INFO("Everything received");
                break;
            }
        }

        tree.push_back(init);

    }
    void obs_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
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
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& data)
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
    void a1_pose_callback(const nav_msgs::Odometry::ConstPtr& data)
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
    ~Planner()
    {
        ROS_INFO("Shutting down");
    }
};

int main(int argc,char **argv)
{
    ros::init(argc, argv, "rrt_planner");
    Planner planner;
    return 0;
}