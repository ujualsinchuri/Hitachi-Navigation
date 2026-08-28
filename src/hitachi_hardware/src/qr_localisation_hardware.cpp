#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class ArucoLocalisation : public rclcpp::Node
{
public:
    ArucoLocalisation() : Node("aruco_localisation")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        aruco_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/aruco_estimated_pose", 10);

        timer = this->create_wall_timer(100ms, std::bind(&ArucoLocalisation::checkPosition, this));

        RCLCPP_INFO(this->get_logger(), "ArucoLocalisation node started. Publishing to /aruco_estimated_pose");
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
            // marker not visible this cycle
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

        if (have_odom) {
            tf2::Vector3 p = odom_to_robot_wheel.getOrigin();
            double yaw = tf2::getYaw(odom_to_robot_wheel.getRotation());
            RCLCPP_INFO(this->get_logger(), "[odom] Position: (%.2f, %.2f), yaw: %.2f deg",
                        p.x(), p.y(), yaw * 180.0 / M_PI);
        }
        if (have_camera_estimate) {
            tf2::Vector3 p = map_to_robot_camera.getOrigin();
            double yaw = tf2::getYaw(map_to_robot_camera.getRotation());
            RCLCPP_INFO(this->get_logger(), "[aruco] Position: (%.2f, %.2f), yaw: %.2f deg",
                        p.x(), p.y(), yaw * 180.0 / M_PI);
        }
        if (have_odom && have_camera_estimate) {
            double pos_diff = (map_to_robot_camera.getOrigin() - odom_to_robot_wheel.getOrigin()).length();

            double yaw_cam  = tf2::getYaw(map_to_robot_camera.getRotation());
            double yaw_odom = tf2::getYaw(odom_to_robot_wheel.getRotation());
            double yaw_diff = yaw_cam - yaw_odom;
            while (yaw_diff > M_PI)  yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

            RCLCPP_INFO(this->get_logger(), "[diff] pos=%.2f m, yaw=%.2f deg",
                        pos_diff, yaw_diff * 180.0 / M_PI);
        }

        if (have_camera_estimate) {
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            // fixed: this is a map-frame pose, not odom-frame -- mislabeling this
            // creates a circular dependency for a map-world-frame EKF and can
            // silently prevent corrections from ever being applied
            pose_msg.header.frame_id = "map";
            pose_msg.header.stamp = this->get_clock()->now();

            pose_msg.pose.pose.position.x = map_to_robot_camera.getOrigin().x();
            pose_msg.pose.pose.position.y = map_to_robot_camera.getOrigin().y();
            pose_msg.pose.pose.position.z = map_to_robot_camera.getOrigin().z();

            pose_msg.pose.pose.orientation.x = map_to_robot_camera.getRotation().x();
            pose_msg.pose.pose.orientation.y = map_to_robot_camera.getRotation().y();
            pose_msg.pose.pose.orientation.z = map_to_robot_camera.getRotation().z();
            pose_msg.pose.pose.orientation.w = map_to_robot_camera.getRotation().w();

            pose_msg.pose.covariance.fill(0.0);
            pose_msg.pose.covariance[0]  = 0.01;  // x
            pose_msg.pose.covariance[7]  = 0.01;  // y
            pose_msg.pose.covariance[14] = 1e5;   // z, unused
            pose_msg.pose.covariance[21] = 1e5;   // roll, unused
            pose_msg.pose.covariance[28] = 1e5;   // pitch, unused
            pose_msg.pose.covariance[35] = 0.01;  // yaw

            aruco_pose_pub_->publish(pose_msg);
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr aruco_pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoLocalisation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}