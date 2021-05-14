
#ifndef VISION_HPP
#define VISION_HPP

//#define OCV_ARUCO

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "image_transport/image_transport.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
//#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

//#ifdef OCV_ARUCO
//#include <opencv2/aruco.hpp>
//#include "dictionary.hpp"
//#else
#include <aruco.h>
//#endif

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>


class Dots_process_cam : public rclcpp::Node
{
public:
    Dots_process_cam(std::string name);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr    img_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       img_pub;

    void img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    const double k1 = -0.1;
    const double k2 =  0.0;
    const double k3 = -0.005;
    const double p1 = -0.002;
    const double p2 =  0.0;
    
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 330, 0, 320, 0, 330, 240, 0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);



#ifdef OCV_ARUCO
    cv::Ptr<cv::aruco::Dictionary>          dictionary;
    std::vector<int>                        marker_ids;
    std::vector<std::vector<cv::Point2f>>   marker_corners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters>  parameters;
#else
    aruco::MarkerDetector                   detector;
#endif

    std::string                             frame_prefix;
    std::string                             cam_name;
    std::string                             cam_cal;
    std::string                             cam_link_frame;


    std::shared_ptr<tf2_ros::TransformBroadcaster>  br;

    std::map<int, double>                   msizes;
};



#endif