#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float32, String


class LocalizationMonitor(Node):

    def __init__(self):
        super().__init__("localization_monitor")

        self.declare_parameter(
            "localized_threshold",
            0.30,
        )

        self.threshold = float(
            self.get_parameter(
                "localized_threshold"
            ).value
        )

        self.robot_x = None
        self.robot_y = None

        self.marker_x = None
        self.marker_y = None

        self.marker_id = None

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.create_subscription(
            Int32,
            "/aruco_id",
            self.id_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/aruco_pose",
            self.pose_callback,
            10,
        )

        self.distance_pub = self.create_publisher(
            Float32,
            "/localization_distance",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/localization_status",
            10,
        )

        self.get_logger().info(
            "Localization monitor started"
        )

    def odom_callback(self, msg):

        self.robot_x = (
            msg.pose.pose.position.x
        )

        self.robot_y = (
            msg.pose.pose.position.y
        )

        self.calculate_distance()

    def id_callback(self, msg):

        self.marker_id = int(
            msg.data
        )

    def pose_callback(self, msg):

        self.marker_x = (
            msg.pose.position.x
        )

        self.marker_y = (
            msg.pose.position.y
        )

        self.calculate_distance()

    def calculate_distance(self):

        if (
            self.robot_x is None
            or self.robot_y is None
            or self.marker_x is None
            or self.marker_y is None
            or self.marker_id is None
        ):
            return

        dx = (
            self.marker_x
            - self.robot_x
        )

        dy = (
            self.marker_y
            - self.robot_y
        )

        distance = math.sqrt(
            dx * dx
            + dy * dy
        )

        distance_msg = Float32()
        distance_msg.data = float(
            distance
        )

        self.distance_pub.publish(
            distance_msg
        )

        status_msg = String()

        if distance <= self.threshold:

            status_msg.data = (
                f"LOCALIZED | "
                f"ID={self.marker_id} | "
                f"distance={distance:.3f} m"
            )

        else:

            status_msg.data = (
                f"APPROACHING | "
                f"ID={self.marker_id} | "
                f"distance={distance:.3f} m"
            )

        self.status_pub.publish(
            status_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = LocalizationMonitor()

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
