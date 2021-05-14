#include <cstdio>

#include "dots_vision/vision.hpp"

using std::placeholders::_1;


//https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
#include <memory>
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}



// Data on image topic is processed using the Aruco library and any tags
// identified are output as an annotated image and as ??



Dots_process_cam::Dots_process_cam(std::string name) 
: Node(name)
{

    frame_prefix    = this->declare_parameter("frame_prefix", "robot_deadbeef_");
    cam_name        = this->declare_parameter("cam_name", "camera0");
    cam_link_frame  = frame_prefix + cam_name + "_link_optical";

    cam_cal         = this->declare_parameter("cam_cal", "cam0.yaml");

    RCLCPP_INFO(this->get_logger(), "cam_frame %s", cam_link_frame.c_str());

    img_sub     = this->create_subscription<sensor_msgs::msg::Image>("img_in", rclcpp::SensorDataQoS(), 
                        std::bind(&Dots_process_cam::img_sub_callback, this, _1));
    img_pub     = this->create_publisher<sensor_msgs::msg::Image>("img_out", rclcpp::SensorDataQoS());

#ifdef OCV_ARUCO
    // Get aruco custom dictionary
    dictionary  = create_dictionary();
    parameters  = cv::aruco::DetectorParameters::create();
#else
    detector.setDictionary("ARUCO_MIP_36h12");
    //aruco::MarkerDetector::Params &parameters = detector.getParameters();
    detector.setDetectionMode(aruco::DM_FAST, 0.01);
#endif

    // Make the transform broadcaster
    br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Marker size map

    msizes[0] = 0.24;
    msizes[1] = 0.24;
    msizes[2] = 0.24;
    msizes[3] = 0.24;
    msizes[4] = 0.24;
    msizes[5] = 0.24;
    msizes[6] = 0.24;
    msizes[7] = 0.24;
    msizes[8] = 0.24;
    msizes[9] = 0.24;
    msizes[10] = 0.24;
    msizes[11] = 0.24;
    msizes[100] = 0.04;
    msizes[101] = 0.04;
    msizes[102] = 0.04;
    msizes[103] = 0.04;
    msizes[104] = 0.04;
    msizes[105] = 0.04;
    msizes[200] = 0.0234;
    msizes[201] = 0.0234;

}


void Dots_process_cam::img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "In image callback");


    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

#ifdef OCV_ARUCO
    auto start = std::chrono::steady_clock::now();

    cv::aruco::detectMarkers(cv_ptr->image, dictionary, 
        marker_corners, marker_ids, parameters, rejected);

    auto end = std::chrono::steady_clock::now();
    auto t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_INFO(this->get_logger(), "Detected %3d %d", marker_ids.size(), t);

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.05, camera_matrix, dist_coeffs, rvecs, tvecs);

    for (int i = 0; i < rvecs.size(); ++i) 
    {
        cv::aruco::drawAxis(cv_ptr->image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
    }
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);

#else
    //auto start = std::chrono::steady_clock::now();

    std::vector<aruco::Marker> markers = detector.detect(cv_ptr->image);
    //auto end = std::chrono::steady_clock::now();
    //auto t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    //RCLCPP_INFO(this->get_logger(), "Detected %3d %d", markers.size(), t);

    // Calculate extrinsics outside detector so we can use different size tags
    aruco::CameraParameters cp(camera_matrix, dist_coeffs, cv::Size(640, 480));
    for (auto &m : markers)
    {
        double marker_size = 0.05;
        auto it = msizes.find(m.id);
        if (it != msizes.end())
            marker_size = it->second;


        m.calculateExtrinsics(marker_size, camera_matrix, dist_coeffs);
        //cv::aruco::drawAxis(cv_ptr->image, camera_matrix, dist_coeffs, m.Rvec, m.Tvec, 0.1);
        aruco::CvDrawingUtils::draw3dAxis(cv_ptr->image, m, cp);
        m.draw(cv_ptr->image);

        if (m.isPoseValid())
        {
            //RCLCPP_INFO(this->get_logger(), "Valid pose, marker id %d size %f rv:% 8f % 8f % 8f tv:% 8f % 8f % 8f", m.id, marker_size, 
            //    m.Rvec.at<float>(0,0), m.Rvec.at<float>(1,0), m.Rvec.at<float>(2,0), m.Tvec.at<float>(0,0), m.Tvec.at<float>(1,0), m.Tvec.at<float>(2,0) );
            // Pose is in form of Tvec and Rvec 3 element tuples, representing
            // transform and rotation.
            //
            // We need to turn this into a tf transform, from the camera to the 
            // detected marker..
            //
            // https://stackoverflow.com/questions/46363618/aruco-markers-with-opencv-get-the-3d-corner-coordinates?rq=1
            //
            cv::Mat rot(3, 3, CV_32FC1);
            cv::Rodrigues(m.Rvec, rot);
            tf2::Matrix3x3 rrot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                                rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                                rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));
            tf2::Quaternion q;
            rrot.getRotation(q);
            if (std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w()))
                continue;

            // Get transform from cam to fiducial
            tf2::BufferCore bc;
            geometry_msgs::msg::TransformStamped t;
            t.header.frame_id           = cam_link_frame;
            // FIXME compensate for camera system latency here
            t.header.stamp              = this->now();
            t.child_frame_id            = string_format("fid%03d", m.id);
            t.transform.translation.x   = m.Tvec.at<float>(0, 0);
            t.transform.translation.y   = m.Tvec.at<float>(1, 0);
            t.transform.translation.z   = m.Tvec.at<float>(2, 0);
            t.transform.rotation.x      = q.x();
            t.transform.rotation.y      = q.y();
            t.transform.rotation.z      = q.z();
            t.transform.rotation.w      = q.w();
            bc.setTransform(t, "default_authority", true);

            // Send the transform
            br->sendTransform(t);

        }

    }
    

#endif



    sensor_msgs::msg::Image img_msg;
    cv_ptr->toImageMsg(img_msg);
    img_pub->publish(img_msg);
}


int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);
    auto n = std::make_shared<Dots_process_cam>("dots_process_cam");
    RCLCPP_INFO(n->get_logger(), "Starting node");
    rclcpp::spin(n);
    rclcpp::shutdown();    
}