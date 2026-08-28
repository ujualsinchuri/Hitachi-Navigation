#!/usr/bin/env python3

import time
from typing import Optional, Set

import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class InspectionManager(Node):
    """Stop only once for each ArUco ID."""

    def __init__(self) -> None:
        super().__init__("inspection_manager")

        self.declare_parameter(
            "stop_duration",
            4.0,
        )

        self.stop_duration = float(
            self.get_parameter(
                "stop_duration"
            ).value
        )

        self.pause_publisher = self.create_publisher(
            Bool,
            "/inspection_pause",
            10,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/inspection_status",
            10,
        )

        self.subscription = self.create_subscription(
            Int32,
            "/aruco_id",
            self.marker_callback,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
        )

        self.inspected_ids: Set[int] = set()
        self.active_id: Optional[int] = None
        self.pause_until = 0.0
        self.paused = False

        self.publish_pause(False)

    def marker_callback(self, message: Int32) -> None:
        marker_id = int(message.data)

        if self.paused:
            return

        if marker_id in self.inspected_ids:
            return

        self.inspected_ids.add(marker_id)
        self.active_id = marker_id
        self.pause_until = (
            time.monotonic()
            + self.stop_duration
        )

        self.publish_pause(True)

        status = String()
        status.data = (
            f"STOPPED_AT_ARUCO_{marker_id}"
        )

        self.status_publisher.publish(status)

    def timer_callback(self) -> None:
        if not self.paused:
            return

        if time.monotonic() < self.pause_until:
            return

        finished_id = self.active_id
        self.publish_pause(False)

        status = String()
        status.data = (
            f"CONTINUING_AFTER_ARUCO_{finished_id}"
        )

        self.status_publisher.publish(status)
        self.active_id = None

    def publish_pause(self, pause: bool) -> None:
        message = Bool()
        message.data = bool(pause)
        self.pause_publisher.publish(message)
        self.paused = bool(pause)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_pause(False)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
