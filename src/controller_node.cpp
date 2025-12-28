#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

class ControllerNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber object_sub_;
    ros::Publisher cmd_pub_;

    double kp_angular_ = 0.002;
    double kp_linear_ = 0.001;

    int image_width_ = 640;
    int image_height_ = 480;

public:
    ControllerNode() {
        object_sub_ = nh_.subscribe("/object_position", 1,
                                   &ControllerNode::objectCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        ROS_INFO("Controller node started.");
    }

    void objectCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double error_x = msg->point.x - image_width_ / 2.0;
        double error_y = msg->point.y - image_height_ / 2.0;

        geometry_msgs::Twist cmd;
        cmd.angular.z = -kp_angular_ * error_x;
        cmd.linear.x = -kp_linear_ * error_y;

        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    ControllerNode cn;
    ros::spin();
    return 0;
}
