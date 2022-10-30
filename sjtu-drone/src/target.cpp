#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Target {
    private:
        geometry_msgs::Twist twist_msg;
        ros::Publisher cmd;
        ros::NodeHandle nh;
    public:
        Target() {
            cmd = nh.advertise<geometry_msgs::Twist>("/drone/target", 10, true);
            run();
        }
        void run() {
            while (ros::ok) {
                std::cout << "Enter goal target" << std::endl;
                std::cout << "Enter x: ";
                std::cin >> twist_msg.linear.x;
                std::cout << "Enter y: ";
                std::cin >> twist_msg.linear.y;
                std::cout << "Enter z: ";
                std::cin >> twist_msg.linear.z;
                cmd.publish(twist_msg);
            }
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "target");
    Target target;
}