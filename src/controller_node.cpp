#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class ControllerNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber object_sub_;
    ros::Publisher cmd_pub_;

    double kp_angular_ = 0.002;
    double kp_linear_ = 0.001;

    int image_width_ = 640;
    int image_height_ = 480;
    std::string object_topic_;

public:
    ControllerNode() {
        ros::NodeHandle pnh("~");

        pnh.param("kp_angular", kp_angular_, kp_angular_);
        pnh.param("kp_linear", kp_linear_, kp_linear_);
        pnh.param("image_width", image_width_, image_width_);
        pnh.param("image_height", image_height_, image_height_);
        pnh.param("object_topic", object_topic_, std::string("/object_position"));

        object_sub_ = nh_.subscribe(object_topic_, 1,
                                   &ControllerNode::objectCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        ROS_INFO("Controller node started. Subscribed to %s", object_topic_.c_str());
    }

    void objectCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        // if object not found (NaN coordinates), stop the robot
        if (std::isnan(msg->point.x) || std::isnan(msg->point.y)) {
            geometry_msgs::Twist stop;
            stop.angular.z = 0.0;
            stop.linear.x = 0.0;
            cmd_pub_.publish(stop);
            return;
        }

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
