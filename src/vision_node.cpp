#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/opencv.hpp>
#include <limits>

class VisionNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher object_pub_;

    // configurable via params
    std::string image_topic_;
    std::string output_topic_;
    bool visualize_ = true;
    int area_threshold_ = 500; // minimum contour area to consider

    // HSV thresholds (can be tuned)
    int h_min_ = 0, s_min_ = 120, v_min_ = 70;
    int h_max_ = 10, s_max_ = 255, v_max_ = 255;

public:
    VisionNode() : it_(nh_) {
        ros::NodeHandle pnh("~");

        // topics and parameters
        pnh.param("image_topic", image_topic_, std::string("/camera/image_raw"));
        pnh.param("output_topic", output_topic_, std::string("/object_position"));
        pnh.param("visualize", visualize_, true);

        // HSV thresholds (can be tuned via params)
        pnh.param("h_min", h_min_, h_min_);
        pnh.param("s_min", s_min_, s_min_);
        pnh.param("v_min", v_min_, v_min_);
        pnh.param("h_max", h_max_, h_max_);
        pnh.param("s_max", s_max_, s_max_);
        pnh.param("v_max", v_max_, v_max_);
        pnh.param("area_threshold", area_threshold_, area_threshold_);

        image_sub_ = it_.subscribe(image_topic_, 1,
                                   &VisionNode::imageCallback, this);
        object_pub_ = nh_.advertise<geometry_msgs::PointStamped>(output_topic_, 1);

        if (visualize_) cv::namedWindow("Object Tracking", cv::WINDOW_AUTOSIZE);

        ROS_INFO("Vision node started. Subscribed to %s", image_topic_.c_str());
    }

    ~VisionNode() {
        if (visualize_) cv::destroyAllWindows();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv, mask;

        // small blur to reduce noise
        cv::Mat blurred;
        cv::GaussianBlur(cv_ptr->image, blurred, cv::Size(5, 5), 0);
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv,
                    cv::Scalar(h_min_, s_min_, v_min_),
                    cv::Scalar(h_max_, s_max_, v_max_),
                    mask);

        // clean small noise
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours,
                         cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            ROS_DEBUG("No contours found");
            // publish an explicit 'not found' message (NaN coordinates)
            geometry_msgs::PointStamped nf_msg;
            nf_msg.header = msg->header;
            nf_msg.point.x = std::numeric_limits<double>::quiet_NaN();
            nf_msg.point.y = std::numeric_limits<double>::quiet_NaN();
            nf_msg.point.z = std::numeric_limits<double>::quiet_NaN();
            object_pub_.publish(nf_msg);
            return;
        }

        // Find largest contour (by area) using iterator to avoid extra copy
        auto it = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        if (it == contours.end()) return;

        double area = cv::contourArea(*it);
        if (area < area_threshold_) {
            ROS_DEBUG("Largest contour too small: %f", area);
            geometry_msgs::PointStamped nf_msg;
            nf_msg.header = msg->header;
            nf_msg.point.x = std::numeric_limits<double>::quiet_NaN();
            nf_msg.point.y = std::numeric_limits<double>::quiet_NaN();
            nf_msg.point.z = std::numeric_limits<double>::quiet_NaN();
            object_pub_.publish(nf_msg);
            return;
        }

        const auto& largest_contour = *it;

        cv::Moments m = cv::moments(largest_contour);
        if (std::abs(m.m00) < 1e-6) {
            geometry_msgs::PointStamped nf_msg;
            nf_msg.header = msg->header;
            nf_msg.point.x = std::numeric_limits<double>::quiet_NaN();
            nf_msg.point.y = std::numeric_limits<double>::quiet_NaN();
            nf_msg.point.z = std::numeric_limits<double>::quiet_NaN();
            object_pub_.publish(nf_msg);
            return;
        }

        geometry_msgs::PointStamped obj_msg;
        obj_msg.header = msg->header; // preserve timestamp and frame
        obj_msg.point.x = m.m10 / m.m00;
        obj_msg.point.y = m.m01 / m.m00;
        obj_msg.point.z = 0.0;

        object_pub_.publish(obj_msg);

        // Visualization (optional)
        if (visualize_) {
            cv::circle(cv_ptr->image,
                       cv::Point(obj_msg.point.x, obj_msg.point.y),
                       10, cv::Scalar(0, 255, 0), -1);
            cv::imshow("Object Tracking", cv_ptr->image);
            cv::waitKey(1);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    VisionNode vn;
    ros::spin();
    return 0;
}
