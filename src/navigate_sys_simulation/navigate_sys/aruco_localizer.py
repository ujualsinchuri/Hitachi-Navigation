#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32


class ArucoLocalizer(Node):
    """
    50-marker ArUco localizer.

    /aruco_id:
        publishes once when a marker is clearly visible
        -> use for localization

    /aruco_close_id:
        publishes once when that marker becomes CLOSE
        -> use for inspection stop

    /aruco_visible:
        True while any mapped marker is visible

    /aruco_close_visible:
        True while any mapped marker is close
    """

    def __init__(self):
        super().__init__("aruco_localizer")

        self.declare_parameter(
            "camera_topic",
            "/down_camera/image_raw",
        )
        self.declare_parameter(
            "marker_csv",
            "",
        )
        self.declare_parameter(
            "minimum_marker_bottom_ratio",
            0.50,
        )
        self.declare_parameter(
            "minimum_area_ratio",
            0.0015,
        )

        # Kept for launch-file compatibility.
        self.declare_parameter(
            "same_marker_cooldown",
            1.0,
        )

        self.camera_topic = str(
            self.get_parameter(
                "camera_topic"
            ).value
        )
        self.close_threshold = float(
            self.get_parameter(
                "minimum_marker_bottom_ratio"
            ).value
        )
        self.minimum_area_ratio = float(
            self.get_parameter(
                "minimum_area_ratio"
            ).value
        )

        self.marker_poses = self.load_poses(
            str(
                self.get_parameter(
                    "marker_csv"
                ).value
            )
        )

        self.dictionary = (
            cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50
            )
        )

        if hasattr(
            cv2.aruco,
            "DetectorParameters",
        ):
            self.parameters = (
                cv2.aruco.DetectorParameters()
            )
        else:
            self.parameters = (
                cv2.aruco.DetectorParameters_create()
            )

        if hasattr(
            self.parameters,
            "minMarkerPerimeterRate",
        ):
            self.parameters.minMarkerPerimeterRate = 0.015

        if hasattr(
            self.parameters,
            "adaptiveThreshWinSizeMin",
        ):
            self.parameters.adaptiveThreshWinSizeMin = 3
            self.parameters.adaptiveThreshWinSizeMax = 33
            self.parameters.adaptiveThreshWinSizeStep = 10

        if hasattr(
            self.parameters,
            "cornerRefinementMethod",
        ):
            self.parameters.cornerRefinementMethod = (
                cv2.aruco.CORNER_REFINE_SUBPIX
            )

        self.detector = None

        if hasattr(
            cv2.aruco,
            "ArucoDetector",
        ):
            self.detector = (
                cv2.aruco.ArucoDetector(
                    self.dictionary,
                    self.parameters,
                )
            )

        self.bridge = CvBridge()

        self.id_pub = self.create_publisher(
            Int32,
            "/aruco_id",
            10,
        )
        self.close_id_pub = self.create_publisher(
            Int32,
            "/aruco_close_id",
            10,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/aruco_pose",
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            "/aruco_debug_image",
            10,
        )
        self.visible_pub = self.create_publisher(
            Bool,
            "/aruco_visible",
            10,
        )
        self.close_pub = self.create_publisher(
            Bool,
            "/aruco_close_visible",
            10,
        )

        self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
        )

        # IMPORTANT: separate memory for localization and inspection.
        self.localized_ids = set()
        self.close_triggered_ids = set()

        self.get_logger().info(
            f"ArUco detector listening on {self.camera_topic}"
        )
        self.get_logger().info(
            f"Loaded {len(self.marker_poses)} marker poses"
        )

    def load_poses(self, csv_path):
        poses = {}
        path = Path(csv_path)

        if not path.is_file():
            self.get_logger().error(
                f"Marker CSV not found: {path}"
            )
            return poses

        with path.open(
            "r",
            encoding="utf-8",
        ) as f:
            for row in csv.DictReader(f):
                marker_id = int(
                    row["marker_id"]
                )

                poses[marker_id] = (
                    float(row["x"]),
                    float(row["y"]),
                    float(row["yaw"]),
                )

        return poses

    def detect(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(
                gray
            )

        return cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

    def publish_pose(
        self,
        marker_id,
        header,
    ):
        if marker_id not in self.marker_poses:
            return

        x, y, yaw = self.marker_poses[
            marker_id
        ]

        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )
        pose.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        self.pose_pub.publish(
            pose
        )

    def publish_localization_once(
        self,
        marker_id,
        header,
    ):
        if marker_id in self.localized_ids:
            return

        self.localized_ids.add(
            marker_id
        )

        msg = Int32()
        msg.data = marker_id

        self.id_pub.publish(
            msg
        )

        self.publish_pose(
            marker_id,
            header,
        )

        self.get_logger().info(
            f"LOCALIZATION ID {marker_id}"
        )

    def publish_close_once(
        self,
        marker_id,
    ):
        if marker_id in self.close_triggered_ids:
            return

        self.close_triggered_ids.add(
            marker_id
        )

        msg = Int32()
        msg.data = marker_id

        self.close_id_pub.publish(
            msg
        )

        self.get_logger().info(
            f"CLOSE ID {marker_id}"
        )

    def image_callback(
        self,
        msg,
    ):
        try:
            frame = (
                self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8",
                )
            )
        except Exception:
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY,
        )

        corners, ids, _ = (
            self.detect(
                gray
            )
        )

        debug = frame.copy()

        height, width = (
            frame.shape[:2]
        )

        image_area = float(
            width * height
        )

        trigger_y = int(
            height
            * self.close_threshold
        )

        cv2.line(
            debug,
            (0, trigger_y),
            (width - 1, trigger_y),
            (255, 0, 255),
            2,
        )

        visible_candidates = []
        close_candidates = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                debug,
                corners,
                ids,
            )

            for (
                marker_corners,
                raw_id,
            ) in zip(
                corners,
                ids.flatten(),
            ):
                marker_id = int(
                    raw_id
                )

                if (
                    marker_id
                    not in self.marker_poses
                ):
                    continue

                polygon = (
                    marker_corners
                    .reshape(4, 2)
                    .astype(
                        np.float32
                    )
                )

                area = abs(
                    cv2.contourArea(
                        polygon
                    )
                )

                area_ratio = (
                    area / image_area
                )

                bottom_ratio = (
                    float(
                        np.max(
                            polygon[:, 1]
                        )
                    )
                    / float(height)
                )

                clearly_visible = (
                    area_ratio
                    >= self.minimum_area_ratio
                )

                close = bool(
                    clearly_visible
                    and bottom_ratio
                    >= self.close_threshold
                )

                center = np.mean(
                    polygon,
                    axis=0,
                )

                if clearly_visible:
                    visible_candidates.append(
                        (
                            area,
                            marker_id,
                        )
                    )

                    self.publish_localization_once(
                        marker_id,
                        msg.header,
                    )

                if close:
                    close_candidates.append(
                        (
                            bottom_ratio,
                            area,
                            marker_id,
                        )
                    )

                cv2.putText(
                    debug,
                    (
                        f"ID {marker_id} "
                        f"{'CLOSE' if close else ('VISIBLE' if clearly_visible else 'SMALL')} "
                        f"B={bottom_ratio:.3f}"
                    ),
                    (
                        max(
                            8,
                            int(center[0]) - 100,
                        ),
                        max(
                            24,
                            int(center[1]) - 20,
                        ),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (
                        (0, 255, 0)
                        if close
                        else (0, 165, 255)
                    ),
                    2,
                )

        visible_id = None

        if visible_candidates:
            visible_candidates.sort(
                reverse=True
            )
            visible_id = (
                visible_candidates[0][1]
            )

        close_id = None

        if close_candidates:
            close_candidates.sort(
                reverse=True
            )

            close_id = (
                close_candidates[0][2]
            )

            self.publish_close_once(
                close_id
            )

        visible_msg = Bool()
        visible_msg.data = (
            visible_id is not None
        )

        self.visible_pub.publish(
            visible_msg
        )

        close_msg = Bool()
        close_msg.data = (
            close_id is not None
        )

        self.close_pub.publish(
            close_msg
        )

        cv2.putText(
            debug,
            (
                "VISIBLE="
                + (
                    str(visible_id)
                    if visible_id is not None
                    else "NONE"
                )
                + "  CLOSE="
                + (
                    str(close_id)
                    if close_id is not None
                    else "NONE"
                )
            ),
            (
                12,
                height - 20,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )

        try:
            debug_msg = (
                self.bridge.cv2_to_imgmsg(
                    debug,
                    encoding="bgr8",
                )
            )

            debug_msg.header = (
                msg.header
            )

            self.debug_pub.publish(
                debug_msg
            )

        except Exception:
            pass


def main(args=None):
    rclpy.init(
        args=args
    )

    node = ArucoLocalizer()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
