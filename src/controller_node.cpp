#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <vision_object_tracking/ObjectDetection.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class ControllerNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber object_sub_;
    ros::Publisher cmd_pub_;

    double kp_angular_ = 0.002;
    double kp_linear_ = 0.001;

    double max_linear_ = 0.5;
    double max_angular_ = 1.0;
    double conf_threshold_ = 0.1;

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
        pnh.param("object_topic", object_topic_, std::string("/object_position_detection"));
        pnh.param("max_linear", max_linear_, max_linear_);
        pnh.param("max_angular", max_angular_, max_angular_);
        pnh.param("conf_threshold", conf_threshold_, conf_threshold_);

        object_sub_ = nh_.subscribe(object_topic_, 1,
                       &ControllerNode::objectCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        ROS_INFO("Controller node started. Subscribed to %s", object_topic_.c_str());
    }

    void objectCallback(const vision_object_tracking::ObjectDetection::ConstPtr& msg) {
        // if object not found (NaN coordinates) or low confidence, stop the robot
        double cx = msg->centroid.x;
        double cy = msg->centroid.y;
        double conf = msg->confidence;
        if (std::isnan(cx) || std::isnan(cy) || std::isnan(conf) || conf < conf_threshold_) {
            geometry_msgs::Twist stop;
            stop.angular.z = 0.0;
            stop.linear.x = 0.0;
            cmd_pub_.publish(stop);
            return;
        }

        double error_x = cx - image_width_ / 2.0;
        double error_y = cy - image_height_ / 2.0;

        geometry_msgs::Twist cmd;
        cmd.angular.z = -kp_angular_ * error_x;
        cmd.linear.x = -kp_linear_ * error_y;

        // clamp velocities
        if (cmd.angular.z > max_angular_) cmd.angular.z = max_angular_;
        if (cmd.angular.z < -max_angular_) cmd.angular.z = -max_angular_;
        if (cmd.linear.x > max_linear_) cmd.linear.x = max_linear_;
        if (cmd.linear.x < -max_linear_) cmd.linear.x = -max_linear_;

        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    ControllerNode cn;
    ros::spin();
    return 0;
}
