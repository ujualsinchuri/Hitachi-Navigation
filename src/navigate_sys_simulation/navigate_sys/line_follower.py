#!/usr/bin/env python3

import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


Point = Tuple[float, float]


class LineFollower(Node):
    """
    Stable red-line follower.

    IMPORTANT:
    - This node does NOT stop itself from ArUco IDs.
    - inspection_manager is the ONLY node that decides when to stop.
    - /inspection_pause=True  -> immediate full stop.
    - /inspection_pause=False -> short straight escape, then normal line following.

    This avoids two independent stop timers fighting each other.
    """

    def __init__(self):
        super().__init__("line_follower")

        self.declare_parameter("camera_topic", "/down_camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("max_linear_speed", 0.050)
        self.declare_parameter("min_linear_speed", 0.018)
        self.declare_parameter("max_angular_speed", 0.58)

        self.declare_parameter("kp", 0.95)
        self.declare_parameter("kd", 0.006)
        self.declare_parameter("steering_sign", -1.0)

        self.declare_parameter("straight_deadband", 0.025)
        self.declare_parameter("straight_speed", 0.050)

        self.declare_parameter("minimum_component_area", 20)
        self.declare_parameter("maximum_horizontal_jump", 150.0)
        self.declare_parameter("gap_tolerance_bands", 3)

        # After each completed inspection stop, move straight briefly so
        # the large marker is behind the camera before normal steering resumes.
        self.declare_parameter("resume_straight_duration", 1.00)
        self.declare_parameter("resume_straight_speed", 0.038)
        self.declare_parameter("recovery_angular_speed", 0.10)

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
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )

        self.kp = float(
            self.get_parameter("kp").value
        )
        self.kd = float(
            self.get_parameter("kd").value
        )
        self.steering_sign = float(
            self.get_parameter("steering_sign").value
        )

        self.straight_deadband = float(
            self.get_parameter("straight_deadband").value
        )
        self.straight_speed = float(
            self.get_parameter("straight_speed").value
        )

        self.minimum_component_area = int(
            self.get_parameter("minimum_component_area").value
        )
        self.maximum_horizontal_jump = float(
            self.get_parameter("maximum_horizontal_jump").value
        )
        self.gap_tolerance_bands = int(
            self.get_parameter("gap_tolerance_bands").value
        )

        self.resume_straight_duration = float(
            self.get_parameter("resume_straight_duration").value
        )
        self.resume_straight_speed = float(
            self.get_parameter("resume_straight_speed").value
        )
        self.recovery_angular_speed = float(
            self.get_parameter("recovery_angular_speed").value
        )

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/line_follower/debug_image",
            10,
        )

        self.mask_pub = self.create_publisher(
            Image,
            "/line_follower/red_mask",
            10,
        )

        self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/inspection_pause",
            self.pause_callback,
            10,
        )

        self.paused = False
        self.resume_until = 0.0

        self.filtered_error = 0.0
        self.previous_error = 0.0
        self.filtered_angular = 0.0
        self.last_time = time.monotonic()

        self.last_turn_direction = 1.0
        self.lost_frames = 0

        self.get_logger().info(
            "Line follower started: single stop controller + corner recovery mode."
        )

    def reset_steering(self):
        self.filtered_error = 0.0
        self.previous_error = 0.0
        self.filtered_angular = 0.0
        self.last_time = time.monotonic()
        self.lost_frames = 0

    def pause_callback(self, msg: Bool):
        new_pause = bool(msg.data)

        # Rising edge -> stop immediately.
        if new_pause and not self.paused:
            self.paused = True
            self.resume_until = 0.0
            self.reset_steering()
            self.stop()

            self.get_logger().info(
                "Inspection pause ON -> robot stopped."
            )
            return

        # Falling edge -> deterministic straight restart.
        if (not new_pause) and self.paused:
            self.paused = False

            self.resume_until = (
                time.monotonic()
                + self.resume_straight_duration
            )

            self.reset_steering()

            self.get_logger().info(
                "Inspection pause OFF -> resume straight, then follow line."
            )
            return

        self.paused = new_pause

        if self.paused:
            self.stop()

    def make_red_mask(self, frame):
        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV,
        )

        mask1 = cv2.inRange(
            hsv,
            np.array([0, 105, 65], dtype=np.uint8),
            np.array([13, 255, 255], dtype=np.uint8),
        )

        mask2 = cv2.inRange(
            hsv,
            np.array([167, 105, 65], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )

        mask = cv2.bitwise_or(
            mask1,
            mask2,
        )

        kernel = np.ones(
            (5, 5),
            dtype=np.uint8,
        )

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

        return mask

    def components_in_band(
        self,
        mask,
        ratio,
        half_height=6,
    ) -> List[Point]:

        height, _ = mask.shape[:2]

        cy = int(height * ratio)
        y1 = max(0, cy - half_height)
        y2 = min(height, cy + half_height + 1)

        band = mask[y1:y2, :]

        (
            count,
            _,
            stats,
            centroids,
        ) = cv2.connectedComponentsWithStats(
            band,
            connectivity=8,
        )

        points = []

        for label_id in range(1, count):
            area = int(
                stats[
                    label_id,
                    cv2.CC_STAT_AREA,
                ]
            )

            if area < self.minimum_component_area:
                continue

            points.append(
                (
                    float(
                        centroids[label_id][0]
                    ),
                    float(
                        centroids[label_id][1]
                    ) + y1,
                )
            )

        return points

    def trace_path(
        self,
        mask,
    ) -> Optional[List[Point]]:

        _, width = mask.shape[:2]

        scan_ratios = [
            0.76,
            0.70,
            0.64,
            0.58,
            0.52,
            0.46,
            0.40,
            0.34,
            0.28,
            0.22,
            0.16,
            0.12,
            0.08,
        ]

        path = []
        missing = 0

        for ratio in scan_ratios:
            candidates = self.components_in_band(
                mask,
                ratio,
            )

            if not path:
                if not candidates:
                    continue

                selected = min(
                    candidates,
                    key=lambda p: abs(
                        p[0] - width / 2.0
                    ),
                )

                path.append(selected)
                continue

            if len(path) >= 2:
                dx = (
                    path[-1][0]
                    - path[-2][0]
                )

                predicted_x = (
                    path[-1][0]
                    + dx
                )

            else:
                predicted_x = path[-1][0]

            if not candidates:
                missing += 1

                if missing <= self.gap_tolerance_bands:
                    continue

                break

            selected = min(
                candidates,
                key=lambda p: abs(
                    p[0] - predicted_x
                ),
            )

            if (
                abs(
                    selected[0]
                    - predicted_x
                )
                > self.maximum_horizontal_jump
            ):
                missing += 1

                if missing <= self.gap_tolerance_bands:
                    continue

                break

            missing = 0
            path.append(selected)

        # Even one valid point is useful after a sharp corner or when
        # a nearby ArUco marker temporarily occludes part of the line.
        if len(path) < 1:
            return None

        return path

    def path_target(self, path):
        # Near-robot path points have slightly more weight.
        weights = []

        for i in range(len(path)):
            weights.append(
                max(
                    0.75,
                    1.45 - 0.07 * i,
                )
            )

        total = sum(weights)

        x = sum(
            p[0] * w
            for p, w in zip(path, weights)
        ) / total

        y = sum(
            p[1] * w
            for p, w in zip(path, weights)
        ) / total

        return x, y

    def straight_command(self, speed):
        command = Twist()
        command.linear.x = float(speed)
        command.angular.z = 0.0
        return command

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception:
            return

        mask = self.make_red_mask(frame)
        now = time.monotonic()

        # -----------------------------------------------------
        # 1. Inspection stop has absolute priority.
        # -----------------------------------------------------
        if self.paused:
            command = Twist()
            self.stop()

            self.publish_debug(
                frame,
                mask,
                None,
                None,
                "INSPECTION STOP",
                command,
                msg,
            )
            return

        # -----------------------------------------------------
        # 2. Stable restart after an inspection.
        # -----------------------------------------------------
        if now < self.resume_until:
            command = self.straight_command(
                self.resume_straight_speed
            )

            self.cmd_pub.publish(command)

            self.publish_debug(
                frame,
                mask,
                None,
                None,
                "RESUME STRAIGHT",
                command,
                msg,
            )
            return

        # -----------------------------------------------------
        # 3. Normal line following.
        # -----------------------------------------------------
        _, width = frame.shape[:2]

        path = self.trace_path(mask)

        command = Twist()
        target = None

        if path is not None:
            target = self.path_target(path)

            error = (
                target[0]
                - width / 2.0
            ) / (
                width / 2.0
            )

            self.filtered_error = (
                0.45 * error
                + 0.55 * self.filtered_error
            )

            current_time = time.monotonic()

            dt = max(
                current_time
                - self.last_time,
                0.005,
            )

            derivative = (
                self.filtered_error
                - self.previous_error
            ) / dt

            self.previous_error = (
                self.filtered_error
            )

            self.last_time = current_time

            if (
                abs(
                    self.filtered_error
                )
                <= self.straight_deadband
            ):
                command.linear.x = (
                    self.straight_speed
                )

                command.angular.z = 0.0

                self.filtered_angular = 0.0

                state = "STRAIGHT"

            else:
                target_angular = (
                    self.steering_sign
                    * (
                        self.kp
                        * self.filtered_error
                        + self.kd
                        * derivative
                    )
                )

                target_angular = float(
                    np.clip(
                        target_angular,
                        -self.max_angular_speed,
                        self.max_angular_speed,
                    )
                )

                self.filtered_angular = (
                    0.50 * target_angular
                    + 0.50 * self.filtered_angular
                )

                command.angular.z = (
                    self.filtered_angular
                )

                turn_ratio = min(
                    abs(
                        command.angular.z
                    )
                    / self.max_angular_speed,
                    1.0,
                )

                command.linear.x = (
                    self.max_linear_speed
                    - (
                        self.max_linear_speed
                        - self.min_linear_speed
                    )
                    * turn_ratio
                )

                state = "TRACK"

            if (
                abs(
                    command.angular.z
                )
                > 0.03
            ):
                self.last_turn_direction = (
                    1.0
                    if command.angular.z > 0.0
                    else -1.0
                )

            self.lost_frames = 0

        else:
            self.lost_frames += 1

            if self.lost_frames <= 25:
                # The line can disappear briefly underneath a large marker.
                command.linear.x = 0.014
                command.angular.z = 0.0
                state = "GAP FORWARD"

            elif self.lost_frames <= 70:
                # Continue the last successful turn direction to recover
                # the new line segment after a corner.
                command.linear.x = 0.006
                command.angular.z = (
                    self.last_turn_direction
                    * self.recovery_angular_speed
                )
                state = "CORNER RECOVERY"

            else:
                # Do not permanently stop just because the camera lost the
                # line. Keep searching until the red line is found again.
                command.linear.x = 0.003
                command.angular.z = (
                    self.last_turn_direction
                    * self.recovery_angular_speed
                )
                state = "SEARCH UNTIL LINE FOUND"

        self.cmd_pub.publish(command)

        self.publish_debug(
            frame,
            mask,
            path,
            target,
            state,
            command,
            msg,
        )

    def publish_debug(
        self,
        frame,
        mask,
        path,
        target,
        state,
        command,
        source_msg,
    ):
        debug = frame.copy()

        height, width = frame.shape[:2]

        cv2.line(
            debug,
            (
                width // 2,
                0,
            ),
            (
                width // 2,
                height - 1,
            ),
            (255, 0, 0),
            2,
        )

        if path is not None:
            points = np.array(
                [
                    (
                        int(x),
                        int(y),
                    )
                    for x, y in path
                ],
                dtype=np.int32,
            )

            if len(points) >= 2:
                cv2.polylines(
                    debug,
                    [points],
                    False,
                    (0, 255, 0),
                    3,
                )

            for p in points:
                cv2.circle(
                    debug,
                    tuple(p),
                    5,
                    (0, 255, 255),
                    -1,
                )

        if target is not None:
            cv2.circle(
                debug,
                (
                    int(target[0]),
                    int(target[1]),
                ),
                9,
                (0, 255, 0),
                -1,
            )

        cv2.putText(
            debug,
            state,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            (
                f"v={command.linear.x:.3f} "
                f"w={command.angular.z:+.3f}"
            ),
            (12, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                debug,
                encoding="bgr8",
            )

            mask_msg = self.bridge.cv2_to_imgmsg(
                mask,
                encoding="mono8",
            )

            debug_msg.header = source_msg.header
            mask_msg.header = source_msg.header

            self.debug_pub.publish(debug_msg)
            self.mask_pub.publish(mask_msg)

        except Exception:
            pass

    def stop(self):
        self.cmd_pub.publish(
            Twist()
        )


def main(args=None):
    rclpy.init(args=args)

    node = LineFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
