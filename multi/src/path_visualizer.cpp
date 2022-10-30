#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

class Visualizer {
    private:
        visualization_msgs::Marker points, line_strip, obstacles;
        geometry_msgs::Point p;
        geometry_msgs::PoseArray path_msg;
        geometry_msgs::Pose pose;
        
        ros::Publisher marker_pub, path_pub;
        ros::NodeHandle nh;
        ros::Subscriber sub;

        float THRESHOLD = 0.2;

        double goal_x, last_point_x;
        double goal_y, last_point_y;

    public:
        Visualizer() {
            marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
            path_pub = nh.advertise<geometry_msgs::PoseArray>("path", 1);
            sub = nh.subscribe("/move_base_simple/goal", 1000, &Visualizer::goal_callback, this);

            ros::Duration(15.0).sleep();

            obstacle_definition();

            points_definition();

            line_strip_definition();

            first_point(); //ADD A FIRST POINT TO THE VECTORS

            while (ros::ok()) {
                ros::spinOnce();
            }
        }
        void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
            goal_x = data->pose.position.x;
            goal_y = data->pose.position.y;
            last_point_x = points.points.back().x;
            last_point_y = points.points.back().y;
            ROS_INFO("Last Point: %f, %f", last_point_x, last_point_y);
            ROS_INFO("Goal: %f, %f", goal_x, goal_y);
            if (abs(goal_x - last_point_x) < THRESHOLD && abs(goal_y - last_point_y) < THRESHOLD) {
                ROS_WARN("Point already in the path! Waiting for antoher point...");
            } else {
                if (abs(goal_x) < THRESHOLD && abs(goal_y) < THRESHOLD) {
                    ROS_INFO("Closing Path!");
                    goal_x = 0.0;
                    goal_y = 0.0;
                    add_point(goal_x, goal_y, data);
                    path_pub.publish(path_msg);
                    ROS_INFO("Path Published!");
                } else {
                    ROS_INFO("Adding point to the path!");
                    add_point(goal_x, goal_y, data);
                }
            }
        }
        void add_point(double x, double y, const geometry_msgs::PoseStamped::ConstPtr& data) {
            p.x = x;
            p.y = y;
            p.z = 0;
            points.points.push_back(p);
            line_strip.points.push_back(p);
            marker_pub.publish(points);
            marker_pub.publish(line_strip);

            //Building the vector of the goal position
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = data->pose.position.z;
            pose.orientation.x = data->pose.orientation.x;
            pose.orientation.y = data->pose.orientation.y;
            pose.orientation.z = data->pose.orientation.z;
            pose.orientation.w = data->pose.orientation.w;

            path_msg.poses.push_back(pose);
        }
        void points_definition() {
            points.header.frame_id = "map";
            points.header.stamp = ros::Time::now();
            points.ns = "sphere";
            points.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            points.action = visualization_msgs::Marker::ADD;
            points.scale.x = 0.05;
            points.scale.y = 0.05;
            points.scale.z = 0.05;
            points.color.a = 1.0; // Don't forget to set the alpha!
            points.color.r = 0.0;
            points.color.g = 1.0;
            points.color.b = 0.0;
            //First point in 0,0,0   
        }
        void line_strip_definition() {
            line_strip.header.frame_id = "map";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "line_strip";
            line_strip.id = 1;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.scale.x = 0.02;
            line_strip.color.a = 1.0; // Don't forget to set the alpha!
            line_strip.color.r = 1.0;
            line_strip.color.g = 1.0;
            line_strip.color.b = 0.0;
            //First point in 0,0,0
        }
        void obstacle_definition() {
            obstacles.header.frame_id = "map";
            obstacles.header.stamp = ros::Time::now();
            obstacles.ns = "obstacles";
            obstacles.id = 2;
            obstacles.type = visualization_msgs::Marker::CUBE;
            obstacles.action = visualization_msgs::Marker::ADD;
            obstacles.scale.x = 1;
            obstacles.scale.y = 2;
            obstacles.scale.z = 0.3;
            obstacles.color.a = 1.0; // Don't forget to set the alpha!
            obstacles.color.r = 1.0;
            obstacles.color.g = 1.0;
            obstacles.color.b = 1.0;
            obstacles.pose.position.x = 1.5;
            obstacles.pose.position.y = 0;
            obstacles.pose.position.z = obstacles.scale.z/2;
            obstacles.pose.orientation.x = 0.0;
            obstacles.pose.orientation.y = 0.0;
            obstacles.pose.orientation.z = 0.0;
            obstacles.pose.orientation.w = 1.0;
            marker_pub.publish(obstacles);
        }
        void first_point() {
            p.x = 0;
            p.y = 0;
            p.z = 0;
            points.points.push_back(p);
            line_strip.points.push_back(p);
            marker_pub.publish(points);
            marker_pub.publish(line_strip);
        }
        ~Visualizer() {
            ROS_INFO("Shutting down...");
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_visualizer");
    Visualizer visualizer;
}