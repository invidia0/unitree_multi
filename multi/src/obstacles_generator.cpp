#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

class Obstacles
{
private:
    visualization_msgs::Marker obs1, obs2;

    visualization_msgs::MarkerArray obs_array;

    geometry_msgs::Point p;
    ros::Publisher marker_pub, obs_pub;
    ros::NodeHandle nh;

public:
    Obstacles()
    {
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
        // Publish the obstacles
        obs1.type = obs2.type = visualization_msgs::Marker::CUBE;
        obs1.header.frame_id = obs2.header.frame_id = "map";
        obs1.header.stamp = obs2.header.stamp = ros::Time::now();
        obs1.ns = obs2.ns = "obstacles";
        obs1.action = obs2.action = visualization_msgs::Marker::ADD;

        obs1.id = -100;
        obs2.id = -101;

        obs1.scale.x = 1.0;
        obs1.scale.y = 2.0;
        obs1.scale.z = 0.3;
        obs1.pose.position.x = 2.0;
        obs1.pose.position.y = 0;
        obs1.pose.position.z = obs1.scale.z/2;
        obs1.pose.orientation.x = 0.0;
        obs1.pose.orientation.y = 0.0;
        obs1.pose.orientation.z = 0.0;
        obs1.pose.orientation.w = 1.0;

        obs1.color.a = 1;
        obs1.color.r = obs1.color.g = obs1.color.b = 1;

        obs2.scale.x = 1.0;
        obs2.scale.y = 2.0;
        obs2.scale.z = 0.3;
        obs2.pose.position.x = -2.0;
        obs2.pose.position.y = 0;
        obs2.pose.position.z = obs2.scale.z/2;
        obs2.pose.orientation.x = 0.0;
        obs2.pose.orientation.y = 0.0;
        obs2.pose.orientation.z = 0.0;
        obs2.pose.orientation.w = 1.0;

        obs2.color.a = 1;
        obs2.color.r = obs2.color.g = obs2.color.b = 1;

        obs_array.markers.push_back(obs1);
        obs_array.markers.push_back(obs2);
        
        while (ros::ok())
        {

            marker_pub.publish(obs_array);

            ros::Rate loop_rate(10);
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacles_generator");
    Obstacles obstacles;
}