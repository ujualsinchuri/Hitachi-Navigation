#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <vector>

using namespace std::chrono_literals;

// Fixed empirical correction between a marker's world-file yaw and what
// aruco_read.cpp's detected orientation actually needs to line up correctly.
// Determined once against aruco_0 (world yaw = 0, needed +90 deg in code).
// This is a property of the detection pipeline's convention, not of any
// individual marker, so it's applied uniformly to every marker here.
static constexpr double YAW_CORRECTION = -1.570796;

struct MarkerPose {
    double x, y, z;
    double yaw;  // world-file yaw; YAW_CORRECTION is added automatically
};

class ArucoLocalisation : public rclcpp::Node
{
public:
    ArucoLocalisation() : Node("aruco_localisation")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        aruco_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/aruco_estimated_pose", 10);

        // Known marker poses in the map/world frame -- from your Gazebo world file.
        marker_poses_[0] = {2.51054,  -3.201,     0.006943, 0.0};
        marker_poses_[1] = {2.56241,  -0.582878,  0.006943, 0.0};
        marker_poses_[2] = {2.02958,   1.67102,   0.006943, 0.0};
        marker_poses_[3] = {0.351743,  3.02532,   0.006943, 0.0};
        marker_poses_[4] = {-1.85735,  2.26292,   0.006943, 0.0};
        marker_poses_[5] = {-3.0818,  -1.22764,   0.006943, 0.0};
        marker_poses_[6] = {-3.30187,  0.931401,  0.006943, 0.0};
        marker_poses_[7] = {-2.70917, -3.47177,   0.006943, 0.0};
        marker_poses_[8] = {-1.42631, -5.30261,   0.006943, 0.0};
        marker_poses_[9] = {0.923339, -5.13537,   0.006943, 0.0};

        timer = this->create_wall_timer(100ms, std::bind(&ArucoLocalisation::checkPosition, this));

        RCLCPP_INFO(this->get_logger(), "ArucoLocalisation node started with %zu known markers. Publishing to /aruco_estimated_pose",
                    marker_poses_.size());
    }

private:
    void checkPosition()
    {
        tf2::Transform map_to_robot_camera;
        bool have_camera_estimate = false;
        int seen_marker_id = -1;
        rclcpp::Time best_stamp(0, 0, this->get_clock()->get_clock_type());

        // Try every known marker each cycle, but don't just take the first
        // ID that resolves -- TF keeps old transforms resolvable for several
        // seconds after a marker leaves view, so an early low-ID marker can
        // keep "winning" long after the robot has moved on to a different
        // one. Instead, check every candidate's actual timestamp and only
        // trust the freshest one that's recent enough to be a real, current
        // detection (not stale cached data).
        const double MAX_AGE_SEC = 0.3;  // aruco_read publishes at ~30Hz, so
                                          // a real detection should be well
                                          // under this age

        for (const auto &entry : marker_poses_) {
            int id = entry.first;
            const MarkerPose &mp = entry.second;
            std::string marker_frame = "aruco_" + std::to_string(id);

            try {
                geometry_msgs::msg::TransformStamped marker_to_robot_msg =
                    tf_buffer_->lookupTransform(marker_frame, "base_footprint", tf2::TimePointZero);

                rclcpp::Time stamp(marker_to_robot_msg.header.stamp, this->get_clock()->get_clock_type());
                double age = (this->get_clock()->now() - stamp).seconds();

                if (age > MAX_AGE_SEC) {
                    // stale cached data from a marker no longer in view -- skip it
                    continue;
                }

                // keep only the most recent candidate seen this cycle
                if (!have_camera_estimate || stamp > best_stamp) {
                    tf2::Transform marker_to_robot_tf2;
                    tf2::fromMsg(marker_to_robot_msg.transform, marker_to_robot_tf2);

                    tf2::Transform map_to_marker;
                    map_to_marker.setOrigin(tf2::Vector3(mp.x, mp.y, mp.z));

                    tf2::Quaternion q_marker;
                    q_marker.setRPY(0, 0, mp.yaw + YAW_CORRECTION);
                    map_to_marker.setRotation(q_marker);

                    map_to_robot_camera = map_to_marker * marker_to_robot_tf2;
                    have_camera_estimate = true;
                    seen_marker_id = id;
                    best_stamp = stamp;
                }
            }
            catch (const tf2::TransformException &ex) {
                // this particular marker not visible this cycle -- try the next
            }
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

        // Hard sanity bound: reject any correction that lands outside the
        // physically possible map extent. A single bad detection (motion
        // blur, misidentified corners, marker too small in-frame) can
        // otherwise get fused with high confidence and lock the filter onto
        // a wrong belief it can never recover from -- this catches that
        // before it's even treated as a real detection.
        if (have_camera_estimate) {
            const double MAP_X_MIN = -4.0, MAP_X_MAX = 3.5;
            const double MAP_Y_MIN = -6.0, MAP_Y_MAX = 4.0;

            double px = map_to_robot_camera.getOrigin().x();
            double py = map_to_robot_camera.getOrigin().y();

            if (px < MAP_X_MIN || px > MAP_X_MAX || py < MAP_Y_MIN || py > MAP_Y_MAX) {
                RCLCPP_WARN(this->get_logger(),
                    "[aruco_%d] Rejected implausible correction: (%.2f, %.2f) is outside map bounds",
                    seen_marker_id, px, py);
                have_camera_estimate = false;
            }
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
            RCLCPP_INFO(this->get_logger(), "[aruco_%d] Position: (%.2f, %.2f), yaw: %.2f deg",
                        seen_marker_id, p.x(), p.y(), yaw * 180.0 / M_PI);
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
    std::map<int, MarkerPose> marker_poses_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoLocalisation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}