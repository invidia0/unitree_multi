#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <map>

class Navigation
{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd;
    ros::Subscriber sub, a1_pose;
    geometry_msgs::Twist twist_msg;
    nav_msgs::Path path_msg;
    double a1_x, a1_y, zPose, yawPose;
    double targetX = 0, targetY = 0;
    double targetYaw;
    double theta_des;
    double ccw_check, cw_check;
    bool aligned = false, path_received = false;
    bool check = false;

    float ANGULAR_SPEED, LINEAR_SPEED;
    float ANGULAR_THRESHOLD = 1 * M_PI / 180;
    float LINEAR_THRESHOLD = 0.2;

public:
    Navigation()
    {
        a1_pose = nh.subscribe("/a1_gazebo/a1/odom", 1, &Navigation::a1_pose_callback, this);
        cmd = nh.advertise<geometry_msgs::Twist>("/a1_gazebo/cmd_vel", 1);
        sub = nh.subscribe("/path", 10, &Navigation::path_callback, this);
        run();
    }
    ~Navigation()
    {
        ROS_INFO("Shutting down...");
    }
    void a1_pose_callback(const nav_msgs::Odometry::ConstPtr &data)
    {
        double roll, pitch, yaw;
        a1_x = data->pose.pose.position.x;
        a1_y = data->pose.pose.position.y;
        tf::Quaternion q(data->pose.pose.orientation.x,
                         data->pose.pose.orientation.y,
                         data->pose.pose.orientation.z,
                         data->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        yawPose = yaw;
        if (yawPose < 0)
        {
            yawPose += 6.28318530718;
        }
    }
    void targetRPY(int i)
    {
        double roll, pitch, yaw;
        targetX = path_msg.poses[i].pose.position.x;
        targetY = path_msg.poses[i].pose.position.y;
        tf::Quaternion q(path_msg.poses[i].pose.orientation.x,
                         path_msg.poses[i].pose.orientation.y,
                         path_msg.poses[i].pose.orientation.z,
                         path_msg.poses[i].pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        targetYaw = yaw;
        if (targetYaw < 0)
        {
            targetYaw += 6.28318530718;
        }
    }
    void path_callback(const nav_msgs::Path::ConstPtr &data)
    {
        if (!path_received)
        {
            path_msg = *data;

            ROS_INFO("Path size: %d", (int)path_msg.poses.size());

            path_received = true;
        }
    }
    void turn(double theta)
    {
        aligned = false;

        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;

        while (true)
        {
            ros::spinOnce();

            align(theta);

            if (yawPose < theta - ANGULAR_THRESHOLD || yawPose > theta + ANGULAR_THRESHOLD)
            {
                ROS_INFO_ONCE("Aligning...");
                twist_msg.angular.z = ANGULAR_SPEED;
                cmd.publish(twist_msg);
            }
            else
            {
                twist_msg.angular.z = 0;
                cmd.publish(twist_msg);
                ROS_INFO("Aligned");
                aligned = true;
                break;
            }
        }
    }
    void desidered_angle()
    {
        theta_des = atan2(targetY - a1_y, targetX - a1_x);
        if (theta_des < 0)
        {
            theta_des += 6.28318530718;
        }
    }
    void turn_direction(double theta)
    {
        int delta = std::fmod(float(theta - yawPose + 2 * M_PI), 2 * M_PI);
        if (delta > M_PI)
        {
            ANGULAR_SPEED = -ANGULAR_SPEED; // Turn CCW
        }
    }
    void align(double theta)
    {

        ANGULAR_SPEED = 0.5 * abs(theta - yawPose);

        if (ANGULAR_SPEED > 1)
        {
            ANGULAR_SPEED = 1;
        }
        else if (ANGULAR_SPEED < -1)
        {
            ANGULAR_SPEED = -1;
        }
        turn_direction(theta);
    }
    void go()
    {
        for (int i = 1; i < path_msg.poses.size(); i++)
        {
            targetRPY(i);

            // desidered_angle();

            // turn(theta_des);

            twist_msg.linear.y = 0;

            double dist;

            ROS_INFO("Moving to target...");

            while (ros::ok)
            {
                ros::spinOnce();

                desidered_angle();

                align(theta_des);
                twist_msg.angular.z = ANGULAR_SPEED;

                dist = sqrt(pow(targetX - a1_x, 2) + pow(targetY - a1_y, 2));

                LINEAR_SPEED = 0.6 * (dist);

                if (LINEAR_SPEED > 1)
                {
                    LINEAR_SPEED = 1;
                }
                else if (LINEAR_SPEED < -1)
                {
                    LINEAR_SPEED = -1;
                }
                else if (LINEAR_SPEED < 0.3)
                {
                    LINEAR_SPEED = 0.3;
                }
                if ((dist) > LINEAR_THRESHOLD)
                {
                    twist_msg.linear.x = LINEAR_SPEED;
                    cmd.publish(twist_msg);
                    ROS_INFO("Going to target...");
                }
                else
                {
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    cmd.publish(twist_msg);
                    aligned = false;
                    break;
                }
            }
        }
        std::cout << "Aligning to final orientation..." << std::endl;
        //turn(targetYaw);
    }
    void run()
    {
        while (ros::ok)
        {
            if (path_received == true)
            {
                go();

                path_received = false;

                ROS_INFO("Path completed!");

                break;
            }
            
            ros::spinOnce(); // Refreshing callbacks
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");
    Navigation nav;
}