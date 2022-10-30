#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class Broadcaster {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        tf::TransformBroadcaster br;
        tf::Transform transform;
    public:
        Broadcaster() {
            sub = nh.subscribe("/a1_gazebo/a1/odom", 1000, &Broadcaster::odom_callback, this);
            transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            while (ros::ok()) {
                ros::spinOnce();     
            }      
        }
        ~Broadcaster() {
            ROS_INFO("Shutting down...");
        }
        void odom_callback(const nav_msgs::Odometry::ConstPtr& data) {
            transform.setOrigin(tf::Vector3(data->pose.pose.position.x, data->pose.pose.position.y, 0.0));
            transform.setRotation(tf::Quaternion(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base"));
            ros::Duration(0.00001).sleep();
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_broadcaster");
    Broadcaster broadcaster;
}