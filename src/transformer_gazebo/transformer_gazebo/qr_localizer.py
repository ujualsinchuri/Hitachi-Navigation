#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


MarkerPose = Tuple[float, float, float]
Detection = Tuple[str, np.ndarray]


class QrLocalizer(Node):
    """Detect QR landmarks and publish their known map poses."""

    def __init__(self) -> None:
        super().__init__("qr_localizer")

        # ROS parameters
        self.declare_parameter(
            "camera_topic",
            "/camera/image_raw",
        )
        self.declare_parameter(
            "marker_csv",
            "",
        )
        self.declare_parameter(
            "upscale_factor",
            2.0,
        )
        self.declare_parameter(
            "detection_cooldown",
            1.0,
        )
        self.declare_parameter(
            "show_processing_method",
            True,
        )

        camera_topic = str(
            self.get_parameter("camera_topic").value
        )

        marker_csv = str(
            self.get_parameter("marker_csv").value
        )

        self.upscale_factor = float(
            self.get_parameter("upscale_factor").value
        )

        self.detection_cooldown = float(
            self.get_parameter("detection_cooldown").value
        )

        self.show_processing_method = bool(
            self.get_parameter(
                "show_processing_method"
            ).value
        )

        # Load QR landmark positions
        self.marker_poses = self.load_marker_poses(
            marker_csv
        )

        # OpenCV and ROS image tools
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        # Publishers
        self.id_publisher = self.create_publisher(
            String,
            "/qr_id",
            10,
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/qr_pose",
            10,
        )

        self.debug_publisher = self.create_publisher(
            Image,
            "/qr_debug_image",
            10,
        )

        # Camera subscriber
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10,
        )

        self.last_marker_id: Optional[str] = None
        self.last_detection_time = 0.0

        self.frame_count = 0

        self.get_logger().info(
            f"QR localizer listening on {camera_topic}"
        )

        self.get_logger().info(
            f"Loaded {len(self.marker_poses)} "
            "QR landmark poses"
        )

        self.get_logger().info(
            f"QR image upscale factor: "
            f"{self.upscale_factor:.1f}"
        )

    def load_marker_poses(
        self,
        csv_path: str,
    ) -> Dict[str, MarkerPose]:
        """Read marker IDs and map poses from a CSV file."""

        marker_poses: Dict[str, MarkerPose] = {}

        if not csv_path:
            self.get_logger().error(
                "The marker_csv parameter is empty."
            )
            return marker_poses

        path = Path(csv_path).expanduser()

        if not path.is_file():
            self.get_logger().error(
                f"QR location CSV was not found: {path}"
            )
            return marker_poses

        try:
            with path.open(
                "r",
                encoding="utf-8",
                newline="",
            ) as csv_file:
                reader = csv.DictReader(csv_file)

                required_columns = {
                    "marker_id",
                    "x",
                    "y",
                    "yaw",
                }

                available_columns = set(
                    reader.fieldnames or []
                )

                missing_columns = (
                    required_columns
                    - available_columns
                )

                if missing_columns:
                    self.get_logger().error(
                        "QR CSV is missing columns: "
                        + ", ".join(
                            sorted(missing_columns)
                        )
                    )
                    return marker_poses

                for row_number, row in enumerate(
                    reader,
                    start=2,
                ):
                    try:
                        marker_id = (
                            row["marker_id"].strip()
                        )

                        if not marker_id:
                            continue

                        marker_poses[marker_id] = (
                            float(row["x"]),
                            float(row["y"]),
                            float(row["yaw"]),
                        )

                    except (
                        KeyError,
                        TypeError,
                        ValueError,
                    ) as exception:
                        self.get_logger().warning(
                            "Could not read CSV row "
                            f"{row_number}: {exception}"
                        )

        except OSError as exception:
            self.get_logger().error(
                f"Could not open QR CSV: {exception}"
            )

        return marker_poses

    def create_processing_images(
        self,
        frame: np.ndarray,
    ) -> List[Tuple[str, np.ndarray]]:
        """
        Create different processed versions of the image.

        QR detection is attempted on every version until one succeeds.
        """

        if self.upscale_factor > 1.0:
            enlarged = cv2.resize(
                frame,
                None,
                fx=self.upscale_factor,
                fy=self.upscale_factor,
                interpolation=cv2.INTER_CUBIC,
            )
        else:
            enlarged = frame.copy()

        gray = cv2.cvtColor(
            enlarged,
            cv2.COLOR_BGR2GRAY,
        )

        # Improve local contrast.
        clahe = cv2.createCLAHE(
            clipLimit=2.5,
            tileGridSize=(8, 8),
        )

        contrast = clahe.apply(gray)

        # Reduce small visual noise while preserving edges.
        blurred = cv2.GaussianBlur(
            contrast,
            (3, 3),
            0,
        )

        # Sharpen QR boundaries.
        sharpened = cv2.addWeighted(
            contrast,
            1.8,
            blurred,
            -0.8,
            0,
        )

        # Automatic global threshold.
        _, otsu = cv2.threshold(
            sharpened,
            0,
            255,
            cv2.THRESH_BINARY
            + cv2.THRESH_OTSU,
        )

        # Adaptive threshold can help when lighting varies.
        adaptive = cv2.adaptiveThreshold(
            sharpened,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            5,
        )

        return [
            ("original", enlarged),
            ("gray", gray),
            ("contrast", contrast),
            ("sharpened", sharpened),
            ("otsu", otsu),
            ("adaptive", adaptive),
        ]

    def detect_multi(
        self,
        image: np.ndarray,
    ) -> List[Detection]:
        """Try detecting multiple QR codes."""

        detections: List[Detection] = []

        try:
            result = (
                self.detector.detectAndDecodeMulti(
                    image
                )
            )

            if len(result) == 4:
                detected, decoded_info, points, _ = (
                    result
                )
            else:
                return detections

            if not detected or points is None:
                return detections

            for marker_id, corners in zip(
                decoded_info,
                points,
            ):
                marker_id = marker_id.strip()

                if not marker_id:
                    continue

                corners_array = np.asarray(
                    corners,
                    dtype=np.float32,
                ).reshape(-1, 2)

                detections.append(
                    (
                        marker_id,
                        corners_array,
                    )
                )

        except cv2.error:
            pass

        return detections

    def detect_single(
        self,
        image: np.ndarray,
    ) -> List[Detection]:
        """Try detecting one QR code."""

        detections: List[Detection] = []

        try:
            marker_id, points, _ = (
                self.detector.detectAndDecode(
                    image
                )
            )

            marker_id = marker_id.strip()

            if marker_id and points is not None:
                corners_array = np.asarray(
                    points,
                    dtype=np.float32,
                ).reshape(-1, 2)

                detections.append(
                    (
                        marker_id,
                        corners_array,
                    )
                )

        except cv2.error:
            pass

        return detections

    def find_qr_codes(
        self,
        frame: np.ndarray,
    ) -> Tuple[
        List[Detection],
        str,
        np.ndarray,
    ]:
        """
        Try several preprocessing methods.

        Returns:
            detections,
            successful method name,
            processed image
        """

        processing_images = (
            self.create_processing_images(frame)
        )

        fallback_name = processing_images[0][0]
        fallback_image = processing_images[0][1]

        for method_name, processed_image in (
            processing_images
        ):
            detections = self.detect_multi(
                processed_image
            )

            if not detections:
                detections = self.detect_single(
                    processed_image
                )

            if detections:
                return (
                    detections,
                    method_name,
                    processed_image,
                )

        return (
            [],
            fallback_name,
            fallback_image,
        )

    def image_callback(
        self,
        message: Image,
    ) -> None:
        """Process each image received from the robot camera."""

        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )

        except Exception as exception:
            self.get_logger().error(
                f"Image conversion failed: "
                f"{exception}"
            )
            return

        self.frame_count += 1

        detections, method_name, processed = (
            self.find_qr_codes(frame)
        )

        # Always publish a colour debug image.
        if len(processed.shape) == 2:
            debug_frame = cv2.cvtColor(
                processed,
                cv2.COLOR_GRAY2BGR,
            )
        else:
            debug_frame = processed.copy()

        if self.show_processing_method:
            cv2.putText(
                debug_frame,
                f"Method: {method_name}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

        if not detections:
            cv2.putText(
                debug_frame,
                "QR: not detected",
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        for marker_id, corners in detections:
            polygon = np.round(
                corners
            ).astype(np.int32)

            cv2.polylines(
                debug_frame,
                [polygon],
                True,
                (0, 255, 0),
                3,
            )

            text_x = int(polygon[0][0])
            text_y = max(
                int(polygon[0][1]) - 10,
                25,
            )

            cv2.putText(
                debug_frame,
                marker_id,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            if marker_id in self.marker_poses:
                self.publish_landmark_pose(
                    marker_id,
                    message,
                )
            else:
                self.get_logger().warning(
                    f"Detected {marker_id}, but it "
                    "was not found in the CSV file."
                )

                # Publish the decoded ID even when it
                # has no corresponding map pose.
                self.publish_marker_id(
                    marker_id
                )

        try:
            debug_message = (
                self.bridge.cv2_to_imgmsg(
                    debug_frame,
                    encoding="bgr8",
                )
            )

            debug_message.header = (
                message.header
            )

            self.debug_publisher.publish(
                debug_message
            )

        except Exception as exception:
            self.get_logger().error(
                "Debug image publishing failed: "
                f"{exception}"
            )

    def should_publish(
        self,
        marker_id: str,
    ) -> bool:
        """Limit repeated messages for the same QR marker."""

        current_time = time.monotonic()

        marker_changed = (
            marker_id != self.last_marker_id
        )

        cooldown_finished = (
            current_time
            - self.last_detection_time
            >= self.detection_cooldown
        )

        if marker_changed or cooldown_finished:
            self.last_marker_id = marker_id
            self.last_detection_time = (
                current_time
            )
            return True

        return False

    def publish_marker_id(
        self,
        marker_id: str,
    ) -> None:
        """Publish a decoded QR identifier."""

        if not self.should_publish(marker_id):
            return

        id_message = String()
        id_message.data = marker_id

        self.id_publisher.publish(
            id_message
        )

        self.get_logger().info(
            f"Detected QR code: {marker_id}"
        )

    def publish_landmark_pose(
        self,
        marker_id: str,
        image_message: Image,
    ) -> None:
        """Publish the known map pose of a QR marker."""

        if not self.should_publish(marker_id):
            return

        x, y, yaw = self.marker_poses[
            marker_id
        ]

        id_message = String()
        id_message.data = marker_id

        self.id_publisher.publish(
            id_message
        )

        pose_message = PoseStamped()

        pose_message.header.stamp = (
            image_message.header.stamp
        )

        pose_message.header.frame_id = "map"

        pose_message.pose.position.x = x
        pose_message.pose.position.y = y
        pose_message.pose.position.z = 0.0

        pose_message.pose.orientation.x = 0.0
        pose_message.pose.orientation.y = 0.0

        pose_message.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )

        pose_message.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        self.pose_publisher.publish(
            pose_message
        )

        self.get_logger().info(
            f"Detected {marker_id}: "
            f"x={x:.2f}, "
            f"y={y:.2f}, "
            f"yaw={yaw:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = QrLocalizer()

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
