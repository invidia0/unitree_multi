#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>

class Converter {
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub1, sub2;
        geometry_msgs::Twist twist;

    public:
        Converter() {
            pub = nh.advertise<geometry_msgs::Twist>("/a1_gazebo/cmd_vel", 1000);
            sub1 = nh.subscribe("/a1_gazebo/joy_ramped", 1000, &Converter::joy_callback, this);
            sub2 = nh.subscribe("/cmd_vel", 1000, &Converter::keyboard_callback, this);
            while (ros::ok()) {
                ros::spinOnce();
            }
        }
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {
            twist.linear.x = joy->axes[1];
            twist.linear.y = joy->axes[0];
            twist.angular.z = joy->axes[3];
            pub.publish(twist);
        }
        void keyboard_callback(const geometry_msgs::Twist::ConstPtr& msg) {
            twist.linear.x = msg->linear.x;
            twist.linear.y = msg->linear.y;
            twist.angular.z = msg->angular.z;
            pub.publish(twist);
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "converter");
    Converter converter;
    return 0;
}
