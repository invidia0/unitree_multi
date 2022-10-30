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
    nav_msgs::Path path;

    double point_x, point_y, point_yaw;
    double robot_x, robot_y, zPose, robot_yaw;
    float dist;
    double targetX = 0, targetY = 0;
    double targetYaw;
    double theta;
    double first_point = true;
    bool aligned = false, path_received = false;
    bool check = false;
    float ANGULAR_SPEED, LINEAR_SPEED_X, LINEAR_SPEED_Y;
    float ANGULAR_THRESHOLD = 2 * M_PI / 180;
    float LINEAR_THRESHOLD = 0.1;

    double path_slope, robot_slope;
    double point_x_next, point_y_next;

public:
    Navigation()
    {
        bringup();

        for (int i = 1; i < path.poses.size(); i++)
        {
            ROS_INFO_ONCE("Path size: %d", (int)path.poses.size());

            point_x = path.poses[i].pose.position.x;
            point_y = path.poses[i].pose.position.y;

            shortest_path();

            while (true)
            {   
                twist_msg.linear.y = 0;

                ros::spinOnce();

                align();

                move();

                if (robot_yaw < theta + ANGULAR_THRESHOLD && robot_yaw > theta - ANGULAR_THRESHOLD)
                {
                    twist_msg.angular.z = 0;
                    cmd.publish(twist_msg);
                }
                else 
                {
                    twist_msg.angular.z = ANGULAR_SPEED;
                    cmd.publish(twist_msg);
                }

                if (dist < LINEAR_THRESHOLD)
                {
                    twist_msg.linear.x = 0;
                    cmd.publish(twist_msg);
                    break;
                }
                else
                {
                    twist_msg.linear.x = LINEAR_SPEED_X;
                    cmd.publish(twist_msg);
                }
            }
        }
    }
    void shortest_path()
    {
        int delta = std::fmod(float(theta - robot_yaw + 2 * M_PI), 2 * M_PI);

        if (delta > M_PI)
        {
            ANGULAR_SPEED = -ANGULAR_SPEED; // Turn CCW
        }     
    }
    void move()
    {
        dist = sqrt(pow(point_x - robot_x, 2) + pow(point_y - robot_y, 2));

        LINEAR_SPEED_X = 0.5 * (dist);
        
        if (LINEAR_SPEED_X > 1)
        {
            LINEAR_SPEED_X = 1;
        }
        else if (LINEAR_SPEED_X < -1)
        {
            LINEAR_SPEED_X = -1;
        }
        else if ((LINEAR_SPEED_X < 0.1 && LINEAR_SPEED_X > 0) || (LINEAR_SPEED_X > -0.1 && LINEAR_SPEED_X < 0))
        {
            LINEAR_SPEED_X = 0;
        }
    }
    void align()
    {
        theta_update();
        // Proporcional control on angular velocity
        ANGULAR_SPEED = 0.5 * abs(theta - robot_yaw);

        if (ANGULAR_SPEED > 1)
        {
            ANGULAR_SPEED = 1;
        }
        else if (ANGULAR_SPEED < -1)
        {
            ANGULAR_SPEED = -1;
        }

        // Find the shortest rotation direction
        // int delta = std::fmod(float(theta - robot_yaw + 2 * M_PI), 2 * M_PI);

        // if (delta > M_PI)
        // {
        //     ANGULAR_SPEED = -ANGULAR_SPEED; // Turn CCW
        // }

        // Rotate the robot
        // twist_msg.angular.z = ANGULAR_SPEED;
        // cmd.publish(twist_msg);
    }
    void theta_update()
    {
        theta = atan2(point_y - robot_y, point_y - robot_x);
        if (theta < 0)
        {
            theta += 6.28318530718;
        }
    }
    void bringup()
    {
        cmd = nh.advertise<geometry_msgs::Twist>("/a1_gazebo/cmd_vel", 1);
        a1_pose = nh.subscribe("/a1_gazebo/a1/odom", 1, &Navigation::a1_pose_callback, this);
        sub = nh.subscribe("/path", 10, &Navigation::path_callback, this);

        while (ros::ok)
        {
            if (path_received == true)
            {
                ROS_INFO_ONCE("Path received");
                break;
            }
            ros::spinOnce(); // Refreshing callbacks
        }
    }
    ~Navigation()
    {
        ROS_INFO("Shutting down...");
    }
    void a1_pose_callback(const nav_msgs::Odometry::ConstPtr &data)
    {
        double roll, pitch, yaw;
        robot_x = data->pose.pose.position.x;
        robot_y = data->pose.pose.position.y;
        tf::Quaternion q(data->pose.pose.orientation.x,
                         data->pose.pose.orientation.y,
                         data->pose.pose.orientation.z,
                         data->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        robot_yaw = yaw;
        if (robot_yaw < 0)
        {
            robot_yaw += 6.28318530718;
        }
    }
    // void targetRPY(int i)
    // {
    //     double roll, pitch, yaw;
    //     targetX = path_msg.poses[i].position.x;
    //     targetY = path_msg.poses[i].position.y;
    //     tf::Quaternion q(path_msg.poses[i].orientation.x,
    //                      path_msg.poses[i].orientation.y,
    //                      path_msg.poses[i].orientation.z,
    //                      path_msg.poses[i].orientation.w);
    //     tf::Matrix3x3 m(q);
    //     m.getRPY(roll, pitch, yaw);
    //     targetYaw = yaw;
    //     if (targetYaw < 0)
    //     {
    //         targetYaw += 6.28318530718;
    //     }
    // }
    void path_callback(const nav_msgs::Path::ConstPtr &data)
    {
        if (!path_received)
        {
            path = *data;

            ROS_INFO("Path size: %d", (int)path.poses.size());
            for (int i = 0; i < 10; i++)
            {
                ROS_INFO("Point %d: (%f, %f)", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
            }
            path_received = true;
        }
    }
    void turn()
    {
        aligned = false;

        twist_msg.linear.x = 0;

        while (true)
        {
            ros::spinOnce();

            align();

            if (robot_yaw < theta - ANGULAR_THRESHOLD || robot_yaw > theta + ANGULAR_THRESHOLD)
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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");
    Navigation nav;
}