#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>

class VisionNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher object_pub_;

    // HSV thresholds (can be tuned)
    int h_min_ = 0, s_min_ = 120, v_min_ = 70;
    int h_max_ = 10, s_max_ = 255, v_max_ = 255;

public:
    VisionNode() : it_(nh_) {
        image_sub_ = it_.subscribe("/camera/image_raw", 1,
                                   &VisionNode::imageCallback, this);
        object_pub_ = nh_.advertise<geometry_msgs::Point>("/object_position", 1);

        ROS_INFO("Vision node started.");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv,
                    cv::Scalar(h_min_, s_min_, v_min_),
                    cv::Scalar(h_max_, s_max_, v_max_),
                    mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours,
                         cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            return;
        }

        // Find largest contour
        auto largest_contour = *std::max_element(
            contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a,
               const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        cv::Moments m = cv::moments(largest_contour);
        if (m.m00 == 0) return;

        geometry_msgs::Point obj_pos;
        obj_pos.x = m.m10 / m.m00;
        obj_pos.y = m.m01 / m.m00;
        obj_pos.z = 0.0;

        object_pub_.publish(obj_pos);

        // Visualization (optional)
        cv::circle(cv_ptr->image,
                   cv::Point(obj_pos.x, obj_pos.y),
                   10, cv::Scalar(0, 255, 0), -1);

        cv::imshow("Object Tracking", cv_ptr->image);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    VisionNode vn;
    ros::spin();
    return 0;
}
