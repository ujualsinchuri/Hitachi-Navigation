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
from std_msgs.msg import Bool


DetectionResult = Tuple[float, int, float, np.ndarray, np.ndarray]


class TransformerLineFollower(Node):
    """Smooth red-line follower for TurtleBot3."""

    def __init__(self) -> None:
        super().__init__("transformer_line_follower")

        self.declare_parameter("camera_topic", "/down_camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("max_linear_speed", 0.045)
        self.declare_parameter("minimum_linear_speed", 0.015)
        self.declare_parameter("maximum_angular_speed", 0.60)
        self.declare_parameter("search_angular_speed", 0.25)

        self.declare_parameter("kp", 0.75)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.035)

        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_deadband", 0.06)

        self.declare_parameter("error_filter_alpha", 0.20)
        self.declare_parameter("derivative_filter_alpha", 0.15)
        self.declare_parameter("angular_filter_alpha", 0.18)

        self.declare_parameter("roi_start_ratio", 0.18)
        self.declare_parameter("lookahead_ratio", 0.48)
        self.declare_parameter("lookahead_band_height", 45)
        self.declare_parameter("minimum_contour_area", 100.0)

        self.declare_parameter("lost_line_search_frames", 80)
        self.declare_parameter("control_timeout", 0.5)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.minimum_linear_speed = float(self.get_parameter("minimum_linear_speed").value)
        self.maximum_angular_speed = float(self.get_parameter("maximum_angular_speed").value)
        self.search_angular_speed = float(self.get_parameter("search_angular_speed").value)

        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)

        self.steering_sign = float(self.get_parameter("steering_sign").value)
        self.steering_deadband = float(self.get_parameter("steering_deadband").value)

        self.error_filter_alpha = float(self.get_parameter("error_filter_alpha").value)
        self.derivative_filter_alpha = float(self.get_parameter("derivative_filter_alpha").value)
        self.angular_filter_alpha = float(self.get_parameter("angular_filter_alpha").value)

        self.roi_start_ratio = float(self.get_parameter("roi_start_ratio").value)
        self.lookahead_ratio = float(self.get_parameter("lookahead_ratio").value)
        self.lookahead_band_height = int(self.get_parameter("lookahead_band_height").value)
        self.minimum_contour_area = float(self.get_parameter("minimum_contour_area").value)

        self.lost_line_search_frames = int(self.get_parameter("lost_line_search_frames").value)
        self.control_timeout = float(self.get_parameter("control_timeout").value)

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

        self.pause_subscription = self.create_subscription(
            Bool,
            "/inspection_pause",
            self.pause_callback,
            10,
        )

        self.safety_timer = self.create_timer(
            0.1,
            self.safety_timer_callback,
        )

        self.paused = False
        self.filtered_error = 0.0
        self.previous_control_error = 0.0
        self.filtered_derivative = 0.0
        self.integral_error = 0.0
        self.filtered_angular_velocity = 0.0

        self.last_control_time: Optional[float] = None
        self.last_image_time = time.monotonic()

        self.lost_line_frames = 0
        self.last_search_direction = 1.0

    def pause_callback(self, message: Bool) -> None:
        self.paused = bool(message.data)

        if self.paused:
            self.reset_controller()
            self.publish_stop()

    def image_callback(self, message: Image) -> None:
        self.last_image_time = time.monotonic()

        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception:
            self.publish_stop()
            return

        if frame is None or frame.size == 0:
            self.publish_stop()
            return

        if self.paused:
            self.publish_stop()
            return

        result = self.detect_red_line(frame)

        if result is None:
            self.handle_lost_line(frame, message)
            return

        line_x, lookahead_y, _, mask, debug_frame = result

        self.lost_line_frames = 0

        height, width = frame.shape[:2]
        image_center_x = width / 2.0

        raw_error = float(
            np.clip(
                (line_x - image_center_x) / image_center_x,
                -1.0,
                1.0,
            )
        )

        self.filtered_error = (
            self.error_filter_alpha * raw_error
            + (1.0 - self.error_filter_alpha) * self.filtered_error
        )

        if abs(self.filtered_error) < self.steering_deadband:
            control_error = 0.0
        else:
            control_error = float(
                np.sign(self.filtered_error)
                * (
                    abs(self.filtered_error) - self.steering_deadband
                )
                / (1.0 - self.steering_deadband)
            )

        current_time = time.monotonic()

        if self.last_control_time is None:
            delta_time = 0.05
        else:
            delta_time = max(
                current_time - self.last_control_time,
                0.001,
            )

        self.last_control_time = current_time

        self.integral_error = float(
            np.clip(
                self.integral_error + control_error * delta_time,
                -0.30,
                0.30,
            )
        )

        raw_derivative = (
            control_error - self.previous_control_error
        ) / delta_time

        self.filtered_derivative = (
            self.derivative_filter_alpha * raw_derivative
            + (1.0 - self.derivative_filter_alpha)
            * self.filtered_derivative
        )

        self.previous_control_error = control_error

        output = (
            self.kp * control_error
            + self.ki * self.integral_error
            + self.kd * self.filtered_derivative
        )

        target_angular = float(
            np.clip(
                self.steering_sign * output,
                -self.maximum_angular_speed,
                self.maximum_angular_speed,
            )
        )

        self.filtered_angular_velocity = (
            self.angular_filter_alpha * target_angular
            + (1.0 - self.angular_filter_alpha)
            * self.filtered_angular_velocity
        )

        if abs(self.filtered_angular_velocity) > 0.03:
            self.last_search_direction = (
                1.0
                if self.filtered_angular_velocity > 0.0
                else -1.0
            )

        slowdown = max(
            min(abs(control_error), 1.0),
            min(
                abs(self.filtered_angular_velocity)
                / self.maximum_angular_speed,
                1.0,
            ),
        )

        linear_velocity = (
            self.max_linear_speed
            - (
                self.max_linear_speed
                - self.minimum_linear_speed
            )
            * slowdown
        )

        command = Twist()
        command.linear.x = float(linear_velocity)
        command.angular.z = float(self.filtered_angular_velocity)

        self.cmd_vel_publisher.publish(command)

        cv2.line(
            debug_frame,
            (int(image_center_x), 0),
            (int(image_center_x), height - 1),
            (255, 0, 0),
            2,
        )

        cv2.circle(
            debug_frame,
            (int(line_x), int(lookahead_y)),
            8,
            (0, 255, 0),
            -1,
        )

        self.publish_debug_images(
            debug_frame,
            mask,
            message,
        )

    def detect_red_line(
        self,
        frame: np.ndarray,
    ) -> Optional[DetectionResult]:
        height, width = frame.shape[:2]

        roi_start_y = int(height * self.roi_start_ratio)
        roi = frame[roi_start_y:height, :]

        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV,
        )

        mask = cv2.bitwise_or(
            cv2.inRange(
                hsv,
                np.array([0, 100, 70], dtype=np.uint8),
                np.array([12, 255, 255], dtype=np.uint8),
            ),
            cv2.inRange(
                hsv,
                np.array([168, 100, 70], dtype=np.uint8),
                np.array([179, 255, 255], dtype=np.uint8),
            ),
        )

        kernel = np.ones((5, 5), dtype=np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        valid = [
            contour
            for contour in contours
            if cv2.contourArea(contour)
            >= self.minimum_contour_area
        ]

        if not valid:
            return None

        contour = max(valid, key=cv2.contourArea)
        contour_area = float(cv2.contourArea(contour))

        contour_mask = np.zeros_like(mask)

        cv2.drawContours(
            contour_mask,
            [contour],
            -1,
            255,
            thickness=cv2.FILLED,
        )

        lookahead_y = int(height * self.lookahead_ratio)
        local_y = lookahead_y - roi_start_y
        half_band = max(self.lookahead_band_height // 2, 1)

        band_top = max(local_y - half_band, 0)
        band_bottom = min(
            local_y + half_band,
            contour_mask.shape[0],
        )

        band_pixels = cv2.findNonZero(
            contour_mask[band_top:band_bottom, :]
        )

        if band_pixels is not None:
            line_x = float(
                np.mean(
                    band_pixels[:, 0, 0]
                )
            )
        else:
            moments = cv2.moments(contour)

            if moments["m00"] <= 0.0:
                return None

            line_x = float(
                moments["m10"] / moments["m00"]
            )

        full_mask = np.zeros(
            (height, width),
            dtype=np.uint8,
        )
        full_mask[roi_start_y:height, :] = mask

        debug_frame = frame.copy()

        display_contour = contour.copy()
        display_contour[:, :, 1] += roi_start_y

        cv2.drawContours(
            debug_frame,
            [display_contour],
            -1,
            (0, 255, 0),
            2,
        )

        cv2.rectangle(
            debug_frame,
            (0, band_top + roi_start_y),
            (width - 1, band_bottom + roi_start_y),
            (255, 255, 0),
            2,
        )

        return (
            line_x,
            lookahead_y,
            contour_area,
            full_mask,
            debug_frame,
        )

    def handle_lost_line(
        self,
        frame: np.ndarray,
        message: Image,
    ) -> None:
        self.lost_line_frames += 1
        self.reset_controller()

        command = Twist()

        if self.lost_line_frames <= self.lost_line_search_frames:
            command.angular.z = (
                self.last_search_direction
                * self.search_angular_speed
            )

        self.cmd_vel_publisher.publish(command)

        empty_mask = np.zeros(
            frame.shape[:2],
            dtype=np.uint8,
        )

        self.publish_debug_images(
            frame,
            empty_mask,
            message,
        )

    def reset_controller(self) -> None:
        self.filtered_error = 0.0
        self.previous_control_error = 0.0
        self.filtered_derivative = 0.0
        self.integral_error = 0.0
        self.filtered_angular_velocity = 0.0
        self.last_control_time = None

    def publish_debug_images(
        self,
        debug_frame: np.ndarray,
        mask: np.ndarray,
        message: Image,
    ) -> None:
        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                debug_frame,
                encoding="bgr8",
            )

            mask_message = self.bridge.cv2_to_imgmsg(
                mask,
                encoding="mono8",
            )

            debug_message.header = message.header
            mask_message.header = message.header

            self.debug_image_publisher.publish(
                debug_message
            )

            self.mask_publisher.publish(
                mask_message
            )

        except Exception:
            pass

    def safety_timer_callback(self) -> None:
        if (
            time.monotonic() - self.last_image_time
            > self.control_timeout
        ):
            self.publish_stop()

    def publish_stop(self) -> None:
        self.cmd_vel_publisher.publish(Twist())

    def destroy_node(self) -> bool:
        self.publish_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TransformerLineFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
