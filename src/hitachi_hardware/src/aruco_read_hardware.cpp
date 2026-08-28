#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sstream>
#include <iomanip>


using namespace std::chrono_literals;

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector() : Node("aruco_detector")
    {
        //create image subcriber.
        image = this->create_subscription<sensor_msgs::msg::Image>(
            "/webcam/image/compressed", 10, std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

        //initialize the tf_broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //setup aruco dictionary and parameters
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        parameters = cv::aruco::DetectorParameters::create();
        
        // web_cam
        // camera_matrix = (cv::Mat_<double>(3, 3) << 
        //     644.122314, 0, 303.697712,
        //     0, 640.593350, 241.197929,
        //     0, 0, 1);
        // dist_coeffs = (cv::Mat_<double>(1, 5) << 0.017622, -0.014850, 0.003126, -0.006205, 0.000000);

        // marker_size = 0.02; // 10 cm

        // gazebo camera
        camera_matrix = (cv::Mat_<double>(3, 3) <<
            1696.802685832259, 0, 960.5,
            0, 1696.802685832259, 540.5,
            0, 0, 1);
        
        dist_coeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        marker_size = 0.1; // 10 cm

        // create display window once (avoid recreating per-frame in callback)
        cv::namedWindow("Aruco Detection", cv::WINDOW_NORMAL);
        cv::resizeWindow("Aruco Detection", 640, 480);

        RCLCPP_INFO(this->get_logger(), "Aruco Detector Node has been started. Waiting for images.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        //detect aruco markers
        cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters, rejected);
        
        if(!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;

            //estimate pose of each marker
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);

            for(size_t i = 0; i < ids.size(); i++) {
                // Process each detected marker
                broadcastArucoFrame(tvecs[i], rvecs[i], ids[i], msg->header.stamp);

                //draw frame axes for each marker
                cv::drawFrameAxes(cv_ptr->image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05);
                std::ostringstream tvec_stream, rvec_stream;
                tvec_stream << std::fixed << std::setprecision(2) << "tvec: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]";
                rvec_stream << std::fixed << std::setprecision(2) << "rvec: [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2] << "]";
                cv::putText(cv_ptr->image, tvec_stream.str(), corners[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                cv::putText(cv_ptr->image, rvec_stream.str(), corners[i][0] + cv::Point2f(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
        }
        cv::imshow("Aruco Detection", cv_ptr->image);
        cv::waitKey(1);
    }

    void broadcastArucoFrame(const cv::Vec3d& tvec, const cv::Vec3d& rvec, int id, const rclcpp::Time& timestamp) {
        
        geometry_msgs::msg::TransformStamped t;

        //header info 
        t.header.stamp = timestamp;
        t.header.frame_id = "camera_optical_frame";
        t.child_frame_id = "aruco_" + std::to_string(id);

        //translation
        t.transform.translation.x = tvec[0];
        t.transform.translation.y = tvec[1];
        t.transform.translation.z = tvec[2];

        //rotation
        double angle = cv::norm(rvec);
        tf2::Quaternion q;
        
        // Convert rotation vector to quaternion
        if (angle > 1e-6) {
            tf2::Vector3 axis(rvec[0]/angle, rvec[1]/angle, rvec[2]/angle);
            q.setRotation(axis, angle);
        } else {
            q.setValue(0, 0, 0, 1); // No rotation
        }

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        //broadcast the transform
        tf_broadcaster->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    double marker_size;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

        