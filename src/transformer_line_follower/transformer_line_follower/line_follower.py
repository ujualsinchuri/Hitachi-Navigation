#!/usr/bin/env python3

import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class TransformerLineFollower(Node):
    """Follow a red floor line using a ROS 2 camera image."""

    def __init__(self) -> None:
        super().__init__("transformer_line_follower")

        # ----------------------------------------------------------
        # ROS parameters
        # ----------------------------------------------------------
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("max_linear_speed", 0.14)
        self.declare_parameter("min_linear_speed", 0.025)
        self.declare_parameter("search_linear_speed", 0.0)

        self.declare_parameter("max_angular_speed", 1.8)
        self.declare_parameter("search_angular_speed", 0.65)

        self.declare_parameter("kp", 2.2)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.30)

        self.declare_parameter("roi_start_ratio", 0.48)
        self.declare_parameter("minimum_contour_area", 120.0)

        self.declare_parameter("lost_line_stop_frames", 120)
        self.declare_parameter("control_timeout", 0.5)

        # Set this to -1.0 if the robot turns in the wrong direction.
        self.declare_parameter("steering_sign", -1.0)

        self.camera_topic = str(
            self.get_parameter("camera_topic").value
        )
        self.cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )

        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.min_linear_speed = float(
            self.get_parameter("min_linear_speed").value
        )
        self.search_linear_speed = float(
            self.get_parameter("search_linear_speed").value
        )

        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.search_angular_speed = float(
            self.get_parameter("search_angular_speed").value
        )

        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)

        self.roi_start_ratio = float(
            self.get_parameter("roi_start_ratio").value
        )
        self.minimum_contour_area = float(
            self.get_parameter("minimum_contour_area").value
        )

        self.lost_line_stop_frames = int(
            self.get_parameter("lost_line_stop_frames").value
        )
        self.control_timeout = float(
            self.get_parameter("control_timeout").value
        )
        self.steering_sign = float(
            self.get_parameter("steering_sign").value
        )

        # ----------------------------------------------------------
        # ROS interfaces
        # ----------------------------------------------------------
        self.bridge = CvBridge()

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            "/line_follower/debug_image",
            10,
        )

        self.mask_publisher = self.create_publisher(
            Image,
            "/line_follower/red_mask",
            10,
        )

        self.image_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
        )

        # Stop the robot if camera images stop arriving.
        self.safety_timer = self.create_timer(
            0.1,
            self.safety_timer_callback,
        )

        # ----------------------------------------------------------
        # Controller state
        # ----------------------------------------------------------
        self.previous_error = 0.0
        self.integral_error = 0.0

        self.last_time: Optional[float] = None
        self.last_image_time = time.monotonic()

        self.lost_line_frames = 0

        # Positive means rotate counterclockwise.
        self.last_search_direction = 1.0

        self.get_logger().info(
            f"Listening for camera images on {self.camera_topic}"
        )
        self.get_logger().info(
            f"Publishing robot commands on {self.cmd_vel_topic}"
        )

    def image_callback(self, message: Image) -> None:
        """Process one camera image and publish a velocity command."""
        self.last_image_time = time.monotonic()

        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception as exception:
            self.get_logger().error(
                f"Could not convert camera image: {exception}"
            )
            self.publish_stop()
            return

        if frame is None or frame.size == 0:
            self.get_logger().warning("Received an empty camera frame.")
            self.publish_stop()
            return

        result = self.detect_red_line(frame)

        if result is None:
            self.handle_lost_line(frame, message)
            return

        line_x, line_y, contour_area, mask, debug_frame = result

        self.lost_line_frames = 0

        height, width = frame.shape[:2]
        image_center_x = width / 2.0

        # Error is negative when line is left and positive when right.
        pixel_error = float(line_x - image_center_x)

        # Normalize approximately into the range -1.0 to +1.0.
        normalized_error = pixel_error / image_center_x
        normalized_error = float(
            np.clip(normalized_error, -1.0, 1.0)
        )

        current_time = time.monotonic()

        if self.last_time is None:
            delta_time = 0.05
        else:
            delta_time = current_time - self.last_time
            delta_time = max(delta_time, 0.001)

        self.last_time = current_time

        # ----------------------------------------------------------
        # PID controller
        # ----------------------------------------------------------
        self.integral_error += normalized_error * delta_time

        # Prevent integral windup.
        self.integral_error = float(
            np.clip(self.integral_error, -1.0, 1.0)
        )

        derivative_error = (
            normalized_error - self.previous_error
        ) / delta_time

        controller_output = (
            self.kp * normalized_error
            + self.ki * self.integral_error
            + self.kd * derivative_error
        )

        angular_velocity = (
            self.steering_sign * controller_output
        )

        angular_velocity = float(
            np.clip(
                angular_velocity,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
        )

        self.previous_error = normalized_error

        # Remember the most recent turning direction.
        if abs(angular_velocity) > 0.05:
            self.last_search_direction = (
                1.0 if angular_velocity > 0.0 else -1.0
            )

        # ----------------------------------------------------------
        # Corner-aware speed control
        # ----------------------------------------------------------
        turn_strength = min(abs(normalized_error), 1.0)

        # Reduce speed progressively when the line moves toward an edge.
        linear_velocity = (
            self.max_linear_speed
            - (
                self.max_linear_speed
                - self.min_linear_speed
            )
            * turn_strength
        )

        # Also slow down when the angular command is large.
        angular_ratio = min(
            abs(angular_velocity) / self.max_angular_speed,
            1.0,
        )

        angular_speed_limit = (
            self.max_linear_speed
            - (
                self.max_linear_speed
                - self.min_linear_speed
            )
            * angular_ratio
        )

        linear_velocity = min(
            linear_velocity,
            angular_speed_limit,
        )

        # At a sharp corner, almost stop forward movement and rotate.
        if abs(normalized_error) > 0.72:
            linear_velocity = self.min_linear_speed

        command = Twist()
        command.linear.x = float(linear_velocity)
        command.angular.z = float(angular_velocity)

        self.cmd_vel_publisher.publish(command)

        # ----------------------------------------------------------
        # Debug annotations
        # ----------------------------------------------------------
        roi_start_y = int(height * self.roi_start_ratio)

        cv2.line(
            debug_frame,
            (int(image_center_x), roi_start_y),
            (int(image_center_x), height - 1),
            (255, 0, 0),
            2,
        )

        cv2.circle(
            debug_frame,
            (line_x, line_y),
            8,
            (0, 255, 0),
            -1,
        )

        cv2.line(
            debug_frame,
            (int(image_center_x), line_y),
            (line_x, line_y),
            (0, 255, 255),
            2,
        )

        cv2.putText(
            debug_frame,
            f"error: {normalized_error:.2f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug_frame,
            f"linear: {linear_velocity:.2f} m/s",
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug_frame,
            f"angular: {angular_velocity:.2f} rad/s",
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug_frame,
            f"area: {contour_area:.0f}",
            (20, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        self.publish_debug_images(
            debug_frame,
            mask,
            message,
        )

    def detect_red_line(
        self,
        frame: np.ndarray,
    ) -> Optional[
        Tuple[int, int, float, np.ndarray, np.ndarray]
    ]:
        """Detect the largest red floor-line contour."""
        height, width = frame.shape[:2]

        roi_start_y = int(height * self.roi_start_ratio)
        roi_start_y = max(0, min(roi_start_y, height - 1))

        roi = frame[roi_start_y:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Red appears at both ends of OpenCV's HSV hue range.
        lower_red_1 = np.array([0, 90, 60], dtype=np.uint8)
        upper_red_1 = np.array([12, 255, 255], dtype=np.uint8)

        lower_red_2 = np.array([168, 90, 60], dtype=np.uint8)
        upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)

        mask_1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1,
        )

        mask_2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2,
        )

        roi_mask = cv2.bitwise_or(mask_1, mask_2)

        kernel = np.ones((5, 5), dtype=np.uint8)

        roi_mask = cv2.morphologyEx(
            roi_mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        roi_mask = cv2.morphologyEx(
            roi_mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        # Slight dilation helps connect small gaps between line segments.
        roi_mask = cv2.dilate(
            roi_mask,
            kernel,
            iterations=1,
        )

        contours, _ = cv2.findContours(
            roi_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        valid_contours = [
            contour
            for contour in contours
            if cv2.contourArea(contour)
            >= self.minimum_contour_area
        ]

        full_mask = np.zeros(
            (height, width),
            dtype=np.uint8,
        )
        full_mask[roi_start_y:height, :] = roi_mask

        debug_frame = frame.copy()

        cv2.rectangle(
            debug_frame,
            (0, roi_start_y),
            (width - 1, height - 1),
            (255, 255, 0),
            2,
        )

        if not valid_contours:
            return None

        largest_contour = max(
            valid_contours,
            key=cv2.contourArea,
        )

        contour_area = float(
            cv2.contourArea(largest_contour)
        )

        moments = cv2.moments(largest_contour)

        if moments["m00"] <= 0.0:
            return None

        centroid_x = int(
            moments["m10"] / moments["m00"]
        )

        centroid_y_roi = int(
            moments["m01"] / moments["m00"]
        )

        centroid_y = centroid_y_roi + roi_start_y

        contour_for_display = largest_contour.copy()
        contour_for_display[:, :, 1] += roi_start_y

        cv2.drawContours(
            debug_frame,
            [contour_for_display],
            -1,
            (0, 255, 0),
            2,
        )

        return (
            centroid_x,
            centroid_y,
            contour_area,
            full_mask,
            debug_frame,
        )

    def handle_lost_line(
        self,
        frame: np.ndarray,
        original_message: Image,
    ) -> None:
        """Rotate toward the last detected line direction."""
        self.lost_line_frames += 1

        # Remove PID history when the line is no longer visible.
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.last_time = None

        command = Twist()

        if self.lost_line_frames <= self.lost_line_stop_frames:
            command.linear.x = self.search_linear_speed
            command.angular.z = (
                self.last_search_direction
                * self.search_angular_speed
            )

            status_text = (
                f"LINE LOST: searching "
                f"{self.lost_line_frames}/"
                f"{self.lost_line_stop_frames}"
            )
        else:
            command.linear.x = 0.0
            command.angular.z = 0.0
            status_text = "LINE LOST: stopped"

        self.cmd_vel_publisher.publish(command)

        debug_frame = frame.copy()

        cv2.putText(
            debug_frame,
            status_text,
            (20, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

        height, width = frame.shape[:2]
        empty_mask = np.zeros(
            (height, width),
            dtype=np.uint8,
        )

        self.publish_debug_images(
            debug_frame,
            empty_mask,
            original_message,
        )

    def publish_debug_images(
        self,
        debug_frame: np.ndarray,
        mask: np.ndarray,
        original_message: Image,
    ) -> None:
        """Publish annotated image and red-line mask."""
        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                debug_frame,
                encoding="bgr8",
            )

            mask_message = self.bridge.cv2_to_imgmsg(
                mask,
                encoding="mono8",
            )

            debug_message.header = original_message.header
            mask_message.header = original_message.header

            self.debug_image_publisher.publish(debug_message)
            self.mask_publisher.publish(mask_message)

        except Exception as exception:
            self.get_logger().warning(
                f"Could not publish debug image: {exception}"
            )

    def safety_timer_callback(self) -> None:
        """Stop the robot if camera data stops arriving."""
        elapsed = time.monotonic() - self.last_image_time

        if elapsed > self.control_timeout:
            self.publish_stop()

    def publish_stop(self) -> None:
        """Publish a zero-velocity command."""
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0
        self.cmd_vel_publisher.publish(command)

    def destroy_node(self) -> bool:
        """Stop the robot before shutting down."""
        self.publish_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TransformerLineFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "Line follower stopped by user."
        )
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
