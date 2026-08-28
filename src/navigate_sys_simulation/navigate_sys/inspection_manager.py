#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path

import rclpy

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from tf2_ros import StaticTransformBroadcaster


class InspectionManager(Node):
    """
    Inspection controller + static ArUco TF landmark publisher.

    Functions:
    - listens to /aruco_close_id
    - stops once for each unique ArUco marker
    - waits for stop_duration
    - resumes the robot
    - publishes a TF frame for every ArUco landmark

    TF structure:

        odom
          |
          +-- aruco_0
          +-- aruco_1
          +-- aruco_2
          ...
          +-- aruco_49
    """

    def __init__(self):
        super().__init__(
            "inspection_manager"
        )

        # -----------------------------------------------------
        # Parameters
        # -----------------------------------------------------

        self.declare_parameter(
            "stop_duration",
            4.0,
        )

        self.declare_parameter(
            "marker_csv",
            "",
        )

        self.declare_parameter(
            "marker_parent_frame",
            "odom",
        )

        self.stop_duration = float(
            self.get_parameter(
                "stop_duration"
            ).value
        )

        self.marker_csv = str(
            self.get_parameter(
                "marker_csv"
            ).value
        )

        self.marker_parent_frame = str(
            self.get_parameter(
                "marker_parent_frame"
            ).value
        )

        # -----------------------------------------------------
        # Publishers
        # -----------------------------------------------------

        self.pause_pub = self.create_publisher(
            Bool,
            "/inspection_pause",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/inspection_status",
            10,
        )

        # -----------------------------------------------------
        # ArUco close-ID subscriber
        # -----------------------------------------------------

        self.create_subscription(
            Int32,
            "/aruco_close_id",
            self.marker_callback,
            10,
        )

        # -----------------------------------------------------
        # Inspection timer
        # -----------------------------------------------------

        self.create_timer(
            0.02,
            self.timer_callback,
        )

        # -----------------------------------------------------
        # Inspection state
        # -----------------------------------------------------

        self.inspected_ids = set()

        self.paused = False

        self.active_id = None

        self.pause_until = 0.0

        # -----------------------------------------------------
        # TF broadcaster
        # -----------------------------------------------------

        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(
                self
            )
        )

        self.marker_poses = (
            self.load_marker_poses(
                self.marker_csv
            )
        )

        self.publish_marker_frames()

        # Initial robot state
        self.publish_pause(
            False
        )

        self.get_logger().info(
            "Inspection manager started."
        )

        self.get_logger().info(
            f"Published "
            f"{len(self.marker_poses)} "
            f"ArUco TF frames."
        )

        self.get_logger().info(
            f"Marker parent frame: "
            f"{self.marker_parent_frame}"
        )

    # =========================================================
    # Load marker positions
    # =========================================================

    def load_marker_poses(
        self,
        csv_path,
    ):

        poses = {}

        path = Path(
            csv_path
        )

        if not path.is_file():

            self.get_logger().error(
                f"ArUco CSV not found: "
                f"{path}"
            )

            return poses

        with path.open(
            "r",
            encoding="utf-8",
        ) as csv_file:

            reader = csv.DictReader(
                csv_file
            )

            for row in reader:

                marker_id = int(
                    row["marker_id"]
                )

                x = float(
                    row["x"]
                )

                y = float(
                    row["y"]
                )

                yaw = float(
                    row["yaw"]
                )

                poses[
                    marker_id
                ] = (
                    x,
                    y,
                    yaw,
                )

        return poses

    # =========================================================
    # Publish TF frames for all markers
    # =========================================================

    def publish_marker_frames(
        self,
    ):

        transforms = []

        for marker_id in sorted(
            self.marker_poses.keys()
        ):

            x, y, yaw = (
                self.marker_poses[
                    marker_id
                ]
            )

            transform = (
                TransformStamped()
            )

            transform.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            transform.header.frame_id = (
                self.marker_parent_frame
            )

            transform.child_frame_id = (
                f"aruco_{marker_id}"
            )

            transform.transform.translation.x = (
                x
            )

            transform.transform.translation.y = (
                y
            )

            transform.transform.translation.z = (
                0.02
            )

            # Convert yaw to quaternion
            transform.transform.rotation.x = (
                0.0
            )

            transform.transform.rotation.y = (
                0.0
            )

            transform.transform.rotation.z = (
                math.sin(
                    yaw / 2.0
                )
            )

            transform.transform.rotation.w = (
                math.cos(
                    yaw / 2.0
                )
            )

            transforms.append(
                transform
            )

        if transforms:

            self.static_tf_broadcaster.sendTransform(
                transforms
            )

    # =========================================================
    # Marker detected close to robot
    # =========================================================

    def marker_callback(
        self,
        msg: Int32,
    ):

        marker_id = int(
            msg.data
        )

        # Do not stop twice for same ID
        if (
            marker_id
            in self.inspected_ids
        ):
            return

        # Ignore new markers while already stopped
        if self.paused:
            return

        self.inspected_ids.add(
            marker_id
        )

        self.active_id = (
            marker_id
        )

        self.paused = True

        self.pause_until = (
            time.monotonic()
            + self.stop_duration
        )

        self.publish_pause(
            True
        )

        status = String()

        status.data = (
            f"STOPPED_AT_"
            f"ARUCO_{marker_id}"
        )

        self.status_pub.publish(
            status
        )

        self.get_logger().info(
            f"ArUco {marker_id} "
            f"-> STOP"
        )

    # =========================================================
    # Inspection timer
    # =========================================================

    def timer_callback(
        self,
    ):

        if not self.paused:
            return

        if (
            time.monotonic()
            < self.pause_until
        ):

            self.publish_pause(
                True
            )

            return

        finished_id = (
            self.active_id
        )

        self.paused = False

        self.active_id = None

        self.pause_until = 0.0

        self.publish_pause(
            False
        )

        status = String()

        status.data = (
            f"CONTINUING_AFTER_"
            f"ARUCO_{finished_id}"
        )

        self.status_pub.publish(
            status
        )

        self.get_logger().info(
            f"ArUco {finished_id} "
            f"-> CONTINUE"
        )

    # =========================================================
    # Publish inspection pause
    # =========================================================

    def publish_pause(
        self,
        value,
    ):

        msg = Bool()

        msg.data = bool(
            value
        )

        self.pause_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = InspectionManager()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.publish_pause(
            False
        )

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()
