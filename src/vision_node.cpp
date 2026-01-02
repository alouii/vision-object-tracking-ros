#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/opencv.hpp>
#include <limits>
#include <dynamic_reconfigure/server.h>
#include <vision_object_tracking/VisionConfig.h>

class VisionNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher object_pub_;
    image_transport::Publisher mask_pub_;

    // configurable via params
    std::string image_topic_;
    std::string output_topic_;
    bool visualize_ = true;
    int area_threshold_ = 500; // minimum contour area to consider
    bool publish_mask_ = false;
    std::string mask_topic_;

    // dynamic reconfigure server
    std::shared_ptr<dynamic_reconfigure::Server<vision_object_tracking::VisionConfig>> dr_srv_;

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
        pnh.param("publish_mask", publish_mask_, publish_mask_);
        pnh.param("mask_topic", mask_topic_, std::string("/vision/mask"));

        image_sub_ = it_.subscribe(image_topic_, 1,
                                   &VisionNode::imageCallback, this);
        object_pub_ = nh_.advertise<geometry_msgs::PointStamped>(output_topic_, 1);
        if (publish_mask_) mask_pub_ = it_.advertise(mask_topic_, 1);

        if (visualize_) cv::namedWindow("Object Tracking", cv::WINDOW_AUTOSIZE);

        // dynamic_reconfigure server
        dr_srv_.reset(new dynamic_reconfigure::Server<vision_object_tracking::VisionConfig>(ros::NodeHandle("~")));
        dynamic_reconfigure::Server<vision_object_tracking::VisionConfig>::CallbackType cb =
            boost::bind(&VisionNode::reconfigCallback, this, _1, _2);
        dr_srv_->setCallback(cb);

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

        // handle hue wrap-around (e.g., red near 0/179 boundary)
        if (h_min_ <= h_max_) {
            cv::inRange(hsv,
                        cv::Scalar(h_min_, s_min_, v_min_),
                        cv::Scalar(h_max_, s_max_, v_max_),
                        mask);
        } else {
            cv::Mat mask1, mask2;
            cv::inRange(hsv,
                        cv::Scalar(h_min_, s_min_, v_min_),
                        cv::Scalar(179, s_max_, v_max_),
                        mask1);
            cv::inRange(hsv,
                        cv::Scalar(0, s_min_, v_min_),
                        cv::Scalar(h_max_, s_max_, v_max_),
                        mask2);
            cv::bitwise_or(mask1, mask2, mask);
        }

        // clean small noise
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // publish mask for debugging
        if (publish_mask_ && mask_pub_.getNumSubscribers() > 0) {
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::MONO8;
            out_msg.image = mask;
            mask_pub_.publish(out_msg.toImageMsg());
        }

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

    void reconfigCallback(vision_object_tracking::VisionConfig &config, uint32_t level) {
        h_min_ = config.h_min;
        s_min_ = config.s_min;
        v_min_ = config.v_min;

        h_max_ = config.h_max;
        s_max_ = config.s_max;
        v_max_ = config.v_max;

        visualize_ = config.visualize;
        area_threshold_ = config.area_threshold;
        // mask publishing
        bool was_publishing = publish_mask_;
        publish_mask_ = config.publish_mask;
        mask_topic_ = config.mask_topic;
        if (publish_mask_ && !was_publishing) {
            mask_pub_ = it_.advertise(mask_topic_, 1);
            ROS_INFO("Mask publishing enabled on %s", mask_topic_.c_str());
        } else if (!publish_mask_ && was_publishing) {
            mask_pub_.shutdown();
            ROS_INFO("Mask publishing disabled");
        } else if (publish_mask_ && was_publishing) {
            // re-advertise if topic changed
            mask_pub_ = it_.advertise(mask_topic_, 1);
            ROS_INFO("Mask topic set to %s", mask_topic_.c_str());
        }

        ROS_INFO("Reconfigured: h[%d..%d] s[%d..%d] v[%d..%d] area=%d vis=%d",
                 h_min_, h_max_, s_min_, s_max_, v_min_, v_max_, area_threshold_, visualize_);
        if (visualize_) cv::namedWindow("Object Tracking", cv::WINDOW_AUTOSIZE);
        else cv::destroyWindow("Object Tracking");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    VisionNode vn;
    ros::spin();
    return 0;
}
