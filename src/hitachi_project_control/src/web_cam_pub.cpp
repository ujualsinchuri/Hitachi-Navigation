#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <sensor_msgs/msg/compressed_image.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("web_cam_pub");
  auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>("/webcam/image/compressed", 10);

  cv::VideoCapture cap(0);

  //fix lag
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

  //low resolution
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  //set fps
  cap.set(cv::CAP_PROP_FPS, 30);

  if (!cap.isOpened()) {
    RCLCPP_ERROR(node->get_logger(), "Unable to open webcam device");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Rate loop_rate(30);
  while (rclcpp::ok()) {
    cv::Mat frame;
    if (!cap.read(frame)) {
      RCLCPP_WARN(node->get_logger(), "Failed to read frame from webcam");
      loop_rate.sleep();
      continue;
    }

    if (frame.empty()) {
      continue;
    }

    std_msgs::msg::Header header;
    header.stamp = node->get_clock()->now();
    header.frame_id = "webcam";

    sensor_msgs::msg::CompressedImage msg;
    msg.header = header;
    msg.format = "jpeg";

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);

    cv::imencode(".jpg", frame, msg.data, compression_params);
    publisher->publish(msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  cap.release();
  rclcpp::shutdown();
  return 0;
}
