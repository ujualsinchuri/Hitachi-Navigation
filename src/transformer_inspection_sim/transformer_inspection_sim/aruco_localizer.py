#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


MarkerPose = Tuple[float, float, float]


class ArucoLocalizer(Node):
    """Publish a marker only when it reaches the close-position trigger line."""

    def __init__(self) -> None:
        super().__init__("aruco_localizer")

        self.declare_parameter(
            "camera_topic",
            "/down_camera/image_raw",
        )
        self.declare_parameter("marker_csv", "")
        self.declare_parameter(
            "dictionary",
            "DICT_4X4_50",
        )
        self.declare_parameter(
            "minimum_marker_bottom_ratio",
            0.58,
        )
        self.declare_parameter(
            "same_marker_cooldown",
            1.0,
        )

        self.camera_topic = str(
            self.get_parameter("camera_topic").value
        )
        self.marker_csv = str(
            self.get_parameter("marker_csv").value
        )
        dictionary_name = str(
            self.get_parameter("dictionary").value
        )
        self.minimum_marker_bottom_ratio = float(
            self.get_parameter(
                "minimum_marker_bottom_ratio"
            ).value
        )
        self.same_marker_cooldown = float(
            self.get_parameter(
                "same_marker_cooldown"
            ).value
        )

        dictionary_id = getattr(
            cv2.aruco,
            dictionary_name,
        )

        self.dictionary = (
            cv2.aruco.getPredefinedDictionary(
                dictionary_id
            )
        )

        if hasattr(
            cv2.aruco,
            "DetectorParameters",
        ):
            parameters = (
                cv2.aruco.DetectorParameters()
            )
        else:
            parameters = (
                cv2.aruco.DetectorParameters_create()
            )

        parameters.minMarkerPerimeterRate = 0.02
        parameters.maxMarkerPerimeterRate = 4.0
        parameters.polygonalApproxAccuracyRate = 0.05

        self.detector = None
        self.parameters = parameters

        if hasattr(
            cv2.aruco,
            "ArucoDetector",
        ):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary,
                self.parameters,
            )

        self.marker_poses = self.load_marker_poses(
            self.marker_csv
        )

        self.bridge = CvBridge()

        self.id_publisher = self.create_publisher(
            Int32,
            "/aruco_id",
            10,
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/aruco_pose",
            10,
        )

        self.debug_publisher = self.create_publisher(
            Image,
            "/aruco_debug_image",
            10,
        )

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
        )

        self.last_marker_id: Optional[int] = None
        self.last_publish_time = 0.0

    def load_marker_poses(
        self,
        csv_path: str,
    ) -> Dict[int, MarkerPose]:
        poses: Dict[int, MarkerPose] = {}
        path = Path(csv_path).expanduser()

        if not path.is_file():
            self.get_logger().error(
                f"Marker CSV not found: {path}"
            )
            return poses

        with path.open("r", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)

            for row in reader:
                poses[int(row["marker_id"])] = (
                    float(row["x"]),
                    float(row["y"]),
                    float(row["yaw"]),
                )

        return poses

    def detect_markers(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)

        return cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

    def image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception:
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY,
        )

        corners, ids, _ = self.detect_markers(gray)
        debug = frame.copy()

        image_height, image_width = frame.shape[:2]
        trigger_y = int(
            image_height
            * self.minimum_marker_bottom_ratio
        )

        cv2.line(
            debug,
            (0, trigger_y),
            (image_width - 1, trigger_y),
            (255, 0, 255),
            2,
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                debug,
                corners,
                ids,
            )

            for marker_corners, raw_id in zip(
                corners,
                ids.flatten(),
            ):
                marker_id = int(raw_id)

                polygon = marker_corners.reshape(
                    4,
                    2,
                ).astype(np.float32)

                bottom_y = float(
                    np.max(polygon[:, 1])
                )

                bottom_ratio = (
                    bottom_y / float(image_height)
                )

                if (
                    bottom_ratio
                    >= self.minimum_marker_bottom_ratio
                    and marker_id in self.marker_poses
                ):
                    self.publish_marker(
                        marker_id,
                        message,
                    )

                    status = (
                        f"ID {marker_id} CLOSE "
                        f"{bottom_ratio:.3f}"
                    )
                    color = (0, 255, 0)
                else:
                    status = (
                        f"ID {marker_id} FAR "
                        f"{bottom_ratio:.3f}"
                    )
                    color = (0, 165, 255)

                centre = np.mean(
                    polygon,
                    axis=0,
                ).astype(int)

                cv2.putText(
                    debug,
                    status,
                    (
                        max(int(centre[0]) - 100, 10),
                        max(int(centre[1]) - 15, 25),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    color,
                    2,
                )

        debug_message = self.bridge.cv2_to_imgmsg(
            debug,
            encoding="bgr8",
        )
        debug_message.header = message.header

        self.debug_publisher.publish(
            debug_message
        )

    def publish_marker(
        self,
        marker_id: int,
        image_message: Image,
    ) -> None:
        now = time.monotonic()

        if (
            marker_id == self.last_marker_id
            and now - self.last_publish_time
            < self.same_marker_cooldown
        ):
            return

        self.last_marker_id = marker_id
        self.last_publish_time = now

        x, y, yaw = self.marker_poses[marker_id]

        id_message = Int32()
        id_message.data = marker_id
        self.id_publisher.publish(id_message)

        pose_message = PoseStamped()
        pose_message.header.stamp = (
            image_message.header.stamp
        )
        pose_message.header.frame_id = "map"

        pose_message.pose.position.x = x
        pose_message.pose.position.y = y
        pose_message.pose.orientation.z = math.sin(
            yaw / 2.0
        )
        pose_message.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        self.pose_publisher.publish(
            pose_message
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
