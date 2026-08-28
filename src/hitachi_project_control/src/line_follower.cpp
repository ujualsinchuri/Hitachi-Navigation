#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

class LineFollower : public rclcpp::Node
{
public:
    LineFollower() : Node("line_follower")
    {   
        //subscribe to the camera topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&LineFollower::imageCallback, this, std::placeholders::_1));

        //publish to the cmd_vel topic
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Line Follower Node has been started. Waiting.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image message to OpenCV image
            cv::Mat cv_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

            // crop image to focus on the lower part
            int h = cv_image.rows;
            int w = cv_image.cols;
            int crop_height = 3.5 * h / 4; 
            int crop_width = w / 4;
            int crop_x = (w - crop_width) / 2;

            //Black out the upper part of the image
            cv::Mat cropped_image = cv_image.clone();
            cv::rectangle(cropped_image, cv::Point(0, 0), cv::Point(w, crop_height), cv::Scalar(0, 0, 0), -1);
            cv::rectangle(cropped_image, cv::Point(0, 0), cv::Point(crop_x, h), cv::Scalar(0, 0, 0), -1);
            cv::rectangle(cropped_image, cv::Point(crop_x + crop_width, h), cv::Point(w, 0), cv::Scalar(0, 0, 0), -1);

            //covert to hsv
            cv::Mat hsv_image;
            cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);
            

            //create mask for red color
            cv::Mat mask1, mask2, mask;
            cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask2);
            cv::bitwise_or(mask1, mask2, mask);

            //find center of mass of red pixel
            cv::Moments m = cv::moments(mask, true);
            auto twist_msg = geometry_msgs::msg::Twist();

            if (m.m00 > 0)
            {
                int cx = m.m10 / m.m00;
                int cy = m.m01 / m.m00;

                // Draw a circle at the center of mass
                cv::circle(cv_image, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);
                // Draw rectangle around the centered lower crop region (from y=crop_height to y=h)
                cv::rectangle(cv_image, cv::Point(crop_x, crop_height), cv::Point(crop_x + crop_width, h), cv::Scalar(0, 255, 0), 2);

                // Propotional control to follow the line
                double kp = 0.0005; // Proportional gain
                double kd = 0.005; // Derivative gain

                double max_linear_speed = 0.15; // Maximum linear speed
                double min_linear_speed = 0.05; // Minimum linear speed
                double max_expected_error = 100.0; // Maximum expected error for linear speed adjustment

                double error = cx - w / 2; 

                double error_diff = error - last_error_;

                double abs_error = std::abs(error);

                if (abs_error > max_expected_error) {
                    abs_error = max_expected_error;
                    
                }
                
                double speed_multiplier = 1.0 - (abs_error / max_expected_error);
                twist_msg.linear.x = min_linear_speed +(max_linear_speed - min_linear_speed) * speed_multiplier;


                RCLCPP_INFO(this->get_logger(), "Error: %f", error);

                twist_msg.angular.z = (-kp * error) - (kd * error_diff);

                RCLCPP_INFO(this->get_logger(), "Twist Command - Linear: %f, Angular: %f", twist_msg.linear.x, twist_msg.angular.z);

                last_error_ = error;

            }

            else
            {
                // If no red line is detected, stop the robot
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.05;
            }
            
            // Publish the Twist message to control the robot
            cmd_pub->publish(twist_msg);

            // Display the image using OpenCV
            cv::namedWindow("Turtlebot Camera Feed", cv::WINDOW_NORMAL);
            cv::resizeWindow("Turtlebot Camera Feed", 640, 480);
            cv::imshow("Turtlebot Camera Feed", cv_image);

            cv::namedWindow("Cropped Image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Cropped Image", 640, 480);
            cv::imshow("Cropped Image", cropped_image);
            cv::waitKey(2);
        }

        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

    double last_error_ = 0.0; 
};

int main(int argc, char *argv[])
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
