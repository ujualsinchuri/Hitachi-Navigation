#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // <-- NEW: Include PoseStamped
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class ArucoVsOdom : public rclcpp::Node
{
public:
    ArucoVsOdom() : Node("aruco_vs_odom")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        //Pose publisher
        aruco_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_estimated_pose", 10);

        timer = this->create_wall_timer(100ms, std::bind(&ArucoVsOdom::checkPosition, this));

        RCLCPP_INFO(this->get_logger(), "Aruco vs Odom comparison node started. Publishing to /aruco_estimated_pose");
    }

private:
    void checkPosition()
    {
        tf2::Transform map_to_robot_camera;
        bool have_camera_estimate = false;

        try {
            geometry_msgs::msg::TransformStamped marker_to_robot_msg =
                tf_buffer_->lookupTransform("aruco_0", "base_footprint", tf2::TimePointZero);

            tf2::Transform marker_to_robot_tf2;
            tf2::fromMsg(marker_to_robot_msg.transform, marker_to_robot_tf2);

            tf2::Transform map_to_marker;
            map_to_marker.setOrigin(tf2::Vector3(2.24272, -4.92731, 0.006943));

            tf2::Quaternion q_marker;
            q_marker.setRPY(0, 0, -1.570796); 
            map_to_marker.setRotation(q_marker);

            map_to_robot_camera = map_to_marker * marker_to_robot_tf2;
            have_camera_estimate = true;
        }
        catch (const tf2::TransformException &ex) {
            // Silencing the warning
        }

        tf2::Transform odom_to_robot_wheel;
        bool have_odom = false;

        try {
            geometry_msgs::msg::TransformStamped odom_msg =
                tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
            tf2::fromMsg(odom_msg.transform, odom_to_robot_wheel);
            have_odom = true;
        }
        catch (const tf2::TransformException &ex) {
        }

        //logging
        if (have_odom && have_camera_estimate) {
            tf2::Vector3 dp = map_to_robot_camera.getOrigin() - odom_to_robot_wheel.getOrigin();
            double pos_diff = dp.length();
            RCLCPP_INFO(this->get_logger(), "Drift Gap: %.2f m", pos_diff);
        }

        // Pose - RVIZ
        if (have_camera_estimate) {
            geometry_msgs::msg::PoseStamped pose_msg;
            //relative to odom
            pose_msg.header.frame_id = "odom"; 
            pose_msg.header.stamp = this->get_clock()->now();

            //X, Y, Z
            pose_msg.pose.position.x = map_to_robot_camera.getOrigin().x();
            pose_msg.pose.position.y = map_to_robot_camera.getOrigin().y();
            pose_msg.pose.position.z = map_to_robot_camera.getOrigin().z();

            //Rotation
            pose_msg.pose.orientation.x = map_to_robot_camera.getRotation().x();
            pose_msg.pose.orientation.y = map_to_robot_camera.getRotation().y();
            pose_msg.pose.orientation.z = map_to_robot_camera.getRotation().z();
            pose_msg.pose.orientation.w = map_to_robot_camera.getRotation().w();

            aruco_pose_pub_->publish(pose_msg);
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_pub_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoVsOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}