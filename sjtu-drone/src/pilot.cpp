#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class DronePilot {
    private:
        ros::NodeHandle nh;
        ros::Publisher takeoff_pub, velMode_pub, cmd;
        ros::Subscriber gt_pose, takeoff, getTarget, a1_pose;
        geometry_msgs::Twist twist_msg;
        std_msgs::Empty empty_msg;
        std_msgs::Bool bool_msg;
        bool check = false;
        double xPose, yPose, zPose, yawPose;
        double targetX = 0, targetY = 0, targetZ = 1;
        double targetYaw;
        bool onTargetX, onTargetY, onTargetZ, onTargetYaw = false;
        bool target_called = false;

        float SPEED = 0.2;
        float THRESHOLD = 0.1;
        
    public:
        DronePilot() {
            takeoff_pub = nh.advertise<std_msgs::Empty>("/drone/takeoff", 10, true);
            velMode_pub = nh.advertise<std_msgs::Bool>("/drone/vel_mode", 10, true);
            cmd = nh.advertise<geometry_msgs::Twist>("/drone/cmd_vel", 10);
            gt_pose = nh.subscribe("/drone/gt_pose", 1000, &DronePilot::get_target_callback, this);
            takeoff = nh.subscribe("/drone/takeoff", 10, &DronePilot::startup_check, this);
            getTarget = nh.subscribe("/drone/target", 10, &DronePilot::target_callback, this);
            a1_pose = nh.subscribe("/a1_gazebo/a1/odom", 1000, &DronePilot::a1_pose_callback, this);
            bool_msg.data = 1;
            init();
            run();
        }
        ~DronePilot() {
            ROS_INFO("Shutting down...");
        }
        void init() {
            takeoff_pub.publish(empty_msg);
            velMode_pub.publish(bool_msg);
            while(!check){
                ROS_INFO("Waiting for startup...");
                ros::Duration(1).sleep();
                ros::spinOnce();
            }
            ROS_INFO("Ok drone is ready to fly");
        }
        void startup_check(const std_msgs::Empty::ConstPtr& msg) {
            ROS_INFO("Startup check...");
            check = true;
        }
        void get_target_callback(const geometry_msgs::Pose::ConstPtr& msg) {
            double fill;
            xPose = msg->position.x;
            yPose = msg->position.y;
            zPose = msg->position.z;
            tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
            tf::Matrix3x3 m(q);
            m.getRPY(fill, fill, yawPose);

        }
        void target_callback(const geometry_msgs::Twist::ConstPtr& data) {
            ROS_INFO("Target received");
            targetX = data->linear.x;
            targetY = data->linear.y;
            //targetZ = data->linear.z;
        }
        void a1_pose_callback(const nav_msgs::Odometry::ConstPtr& data) {
            ROS_INFO_ONCE("Target: Unitree A1");
            double fill;
            targetX = data->pose.pose.position.x;
            targetY = data->pose.pose.position.y;
            tf::Quaternion q(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            m.getRPY(fill, fill, targetYaw);
            std::cout << (targetYaw*180)/3.14 << std::endl;
            //targetZ = data->pose.pose.position.z;
        }
        void correct_x(double x) {
            if(x >= targetX+THRESHOLD) {
                twist_msg.linear.x = -SPEED;
            }
            else if(x <= targetX-THRESHOLD) {
                twist_msg.linear.x = SPEED;
            }
            else {
                ROS_INFO_ONCE("X on target");
                twist_msg.linear.x = 0;
                onTargetX = true;
            }
        }
        void correct_y(double y) {
            if(y >= targetY+THRESHOLD) {
                twist_msg.linear.y = -SPEED;
            }
            else if(y <= targetY-THRESHOLD) {
                twist_msg.linear.y = SPEED;
            }
            else {
                ROS_INFO_ONCE("Y on target");
                twist_msg.linear.y = 0;
                onTargetY = true;
            }          
        }
        void correct_z(double z) {
            if(z >= targetZ+THRESHOLD) {
                twist_msg.linear.z = -SPEED;
            }
            else if(z <= targetZ-THRESHOLD) {
                twist_msg.linear.z = SPEED;
            }
            else {
                ROS_INFO_ONCE("Z Target reached");
                twist_msg.linear.z = 0;
                onTargetZ = true;
            }          
        }
        void correct_yaw(double yaw) {
            if(yaw >= targetYaw+THRESHOLD) {
                twist_msg.angular.z = -SPEED;
            }
            else if(yaw <= targetYaw-THRESHOLD) {
                twist_msg.angular.z = SPEED;
            }
            else {
                ROS_INFO_ONCE("Yaw on target");
                twist_msg.angular.z = 0;
                onTargetYaw = true;
            }
        }
        void hover_correction() {
            ROS_INFO_ONCE("Hovering...");
            correct_x(xPose);
            correct_y(yPose);
            correct_z(zPose);
            correct_yaw(yawPose);
            cmd.publish(twist_msg);
        }
        void run() {
            while(ros::ok){
                ros::spinOnce();
                hover_correction();
                if(onTargetX && onTargetY && onTargetZ) {
                    ROS_INFO_ONCE("Goal reached!");
                    target_called = false;
                }
            }
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_pilot");
    DronePilot();
}
