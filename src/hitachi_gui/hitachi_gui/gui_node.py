#!/usr/bin/env python3

import sys
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
)


# ================================================================
# ROS NODE
# ================================================================

class HitachiGuiNode(Node):

    def __init__(self):
        super().__init__('hitachi_gui')

        # --------------------------------------------------------
        # Images
        # --------------------------------------------------------

        self.aruco_frame = None
        self.line_frame = None
        self.mask_frame = None

        # --------------------------------------------------------
        # ArUco
        # --------------------------------------------------------

        self.aruco_id = None
        self.close_id = None

        self.aruco_visible = False
        self.close_visible = False

        self.distance = None

        # ArUco pose measured relative to the camera.
        # Kept separate from /odom so the GUI does not confuse a
        # marker-relative measurement with the robot's global pose.
        self.aruco_rel_x = None
        self.aruco_rel_y = None
        self.aruco_rel_z = None

        # --------------------------------------------------------
        # System status
        # --------------------------------------------------------

        self.localization_status = 'WAITING'
        self.inspection_status = 'WAITING'
        self.inspection_pause = False

        # --------------------------------------------------------
        # Robot
        # --------------------------------------------------------

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # GUI command state
        self.command_state = 'MONITORING'

        # ========================================================
        # IMAGE SUBSCRIBERS
        # ========================================================

        self.create_subscription(
            Image,
            '/aruco_debug_image',
            self.aruco_image_callback,
            10
        )

        self.create_subscription(
            Image,
            '/line_follower/debug_image',
            self.line_image_callback,
            10
        )

        self.create_subscription(
            Image,
            '/line_follower/red_mask',
            self.mask_image_callback,
            10
        )

        # ========================================================
        # ARUCO SUBSCRIBERS
        # ========================================================

        self.create_subscription(
            Int32,
            '/aruco_id',
            self.aruco_id_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/aruco_close_id',
            self.close_id_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/aruco_visible',
            self.aruco_visible_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/aruco_close_visible',
            self.close_visible_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.aruco_pose_callback,
            10
        )

        # ========================================================
        # LOCALIZATION / INSPECTION
        # ========================================================

        self.create_subscription(
            Float32,
            '/localization_distance',
            self.distance_callback,
            10
        )

        self.create_subscription(
            String,
            '/localization_status',
            self.localization_callback,
            10
        )

        self.create_subscription(
            String,
            '/inspection_status',
            self.inspection_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/inspection_pause',
            self.inspection_pause_callback,
            10
        )

        # ========================================================
        # ROBOT
        # ========================================================

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # ========================================================
        # GUI COMMAND PUBLISHER
        # ========================================================

        self.command_pub = self.create_publisher(
            String,
            '/system_command',
            10
        )

        self.get_logger().info(
            'Hitachi GUI started'
        )

    # ============================================================
    # IMAGE CONVERSION
    # ============================================================

    def ros_image_to_cv(self, msg):

        try:

            if msg.encoding in ['mono8', '8UC1']:

                image = np.frombuffer(
                    msg.data,
                    dtype=np.uint8
                )

                image = image.reshape(
                    msg.height,
                    msg.width
                )

                return image.copy()

            channels = 3

            if msg.encoding in ['rgba8', 'bgra8']:
                channels = 4

            image = np.frombuffer(
                msg.data,
                dtype=np.uint8
            )

            image = image.reshape(
                msg.height,
                msg.width,
                channels
            )

            if msg.encoding == 'rgb8':

                image = cv2.cvtColor(
                    image,
                    cv2.COLOR_RGB2BGR
                )

            elif msg.encoding == 'rgba8':

                image = cv2.cvtColor(
                    image,
                    cv2.COLOR_RGBA2BGR
                )

            elif msg.encoding == 'bgra8':

                image = cv2.cvtColor(
                    image,
                    cv2.COLOR_BGRA2BGR
                )

            return image.copy()

        except Exception as error:

            self.get_logger().warning(
                f'Image conversion error: {error}'
            )

            return None

    # ============================================================
    # IMAGE CALLBACKS
    # ============================================================

    def aruco_image_callback(self, msg):

        self.aruco_frame = self.ros_image_to_cv(msg)

    def line_image_callback(self, msg):

        self.line_frame = self.ros_image_to_cv(msg)

    def mask_image_callback(self, msg):

        self.mask_frame = self.ros_image_to_cv(msg)

    # ============================================================
    # OTHER CALLBACKS
    # ============================================================

    def aruco_id_callback(self, msg):
        self.aruco_id = msg.data

    def close_id_callback(self, msg):
        self.close_id = msg.data

    def aruco_visible_callback(self, msg):
        self.aruco_visible = msg.data

    def close_visible_callback(self, msg):
        self.close_visible = msg.data

    def aruco_pose_callback(self, msg):
        self.aruco_rel_x = msg.pose.position.x
        self.aruco_rel_y = msg.pose.position.y
        self.aruco_rel_z = msg.pose.position.z

    def distance_callback(self, msg):
        self.distance = msg.data

    def localization_callback(self, msg):
        self.localization_status = msg.data

    def inspection_callback(self, msg):
        self.inspection_status = msg.data

    def inspection_pause_callback(self, msg):
        self.inspection_pause = msg.data

    def velocity_callback(self, msg):

        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.robot_yaw = math.degrees(yaw)

    # ============================================================
    # COMMAND
    # ============================================================

    def send_command(self, command):

        msg = String()
        msg.data = command

        self.command_pub.publish(msg)

        self.command_state = command

        self.get_logger().info(
            f'GUI command: {command}'
        )


# ================================================================
# GUI
# ================================================================

class MainWindow(QMainWindow):

    def __init__(self, ros_node):

        super().__init__()

        self.ros_node = ros_node

        self.last_marker = None
        self.last_close_marker = None
        self.last_inspection_status = None

        self.setWindowTitle(
            'Hitachi AMR Transformer Inspection System'
        )

        self.resize(1550, 900)

        # ========================================================
        # STYLE
        # ========================================================

        self.setStyleSheet("""
            QMainWindow {
                background-color: #171a1f;
            }

            QWidget {
                background-color: #171a1f;
            }

            QLabel {
                color: #e8e8e8;
                font-size: 13px;
            }

            QGroupBox {
                color: white;
                font-size: 15px;
                font-weight: bold;
                border: 1px solid #4c566a;
                border-radius: 7px;
                margin-top: 12px;
                padding-top: 14px;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }

            QTextEdit {
                background-color: #101216;
                color: #d8dee9;
                border: 1px solid #4c566a;
                border-radius: 5px;
                font-family: monospace;
            }
        """)

        central = QWidget()

        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)

        # ========================================================
        # HEADER
        # ========================================================

        header = QHBoxLayout()

        title = QLabel(
            'HITACHI AMR TRANSFORMER INSPECTION SYSTEM'
        )

        title.setStyleSheet("""
            font-size: 25px;
            font-weight: bold;
            padding: 10px;
        """)

        self.ros_status = QLabel(
            '● ROS 2 ONLINE'
        )

        self.ros_status.setStyleSheet("""
            color: #4CAF50;
            font-weight: bold;
            font-size: 15px;
        """)

        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.ros_status)

        main_layout.addLayout(header)

        # ========================================================
        # BODY
        # ========================================================

        body = QHBoxLayout()

        main_layout.addLayout(body)

        # ========================================================
        # LEFT VISUALIZATION
        # ========================================================

        visual_layout = QVBoxLayout()

        # --------------------------------------------------------
        # Main ArUco image
        # --------------------------------------------------------

        aruco_group = QGroupBox(
            'ARUCO LOCALIZATION CAMERA'
        )

        aruco_layout = QVBoxLayout(
            aruco_group
        )

        self.aruco_image = self.create_image_label(
            'Waiting for /aruco_debug_image'
        )

        self.aruco_image.setMinimumSize(
            760,
            400
        )

        aruco_layout.addWidget(
            self.aruco_image
        )

        visual_layout.addWidget(
            aruco_group,
            2
        )

        # --------------------------------------------------------
        # Bottom two images
        # --------------------------------------------------------

        lower_visuals = QHBoxLayout()

        line_group = QGroupBox(
            'LINE FOLLOWER'
        )

        line_layout = QVBoxLayout(
            line_group
        )

        self.line_image = self.create_image_label(
            'Waiting for /line_follower/debug_image'
        )

        self.line_image.setMinimumSize(
            370,
            240
        )

        line_layout.addWidget(
            self.line_image
        )

        mask_group = QGroupBox(
            'RED LINE SEGMENTATION'
        )

        mask_layout = QVBoxLayout(
            mask_group
        )

        self.mask_image = self.create_image_label(
            'Waiting for /line_follower/red_mask'
        )

        self.mask_image.setMinimumSize(
            370,
            240
        )

        mask_layout.addWidget(
            self.mask_image
        )

        lower_visuals.addWidget(
            line_group
        )

        lower_visuals.addWidget(
            mask_group
        )

        visual_layout.addLayout(
            lower_visuals
        )

        body.addLayout(
            visual_layout,
            3
        )

        # ========================================================
        # RIGHT CONTROL PANEL
        # ========================================================

        right = QVBoxLayout()

        # ========================================================
        # SYSTEM CONTROL
        # ========================================================

        control_group = QGroupBox(
            'SYSTEM CONTROL'
        )

        control_layout = QVBoxLayout(
            control_group
        )

        self.system_mode = QLabel(
            'MODE: MONITORING'
        )

        self.system_mode.setAlignment(
            Qt.AlignCenter
        )

        self.system_mode.setStyleSheet("""
            font-size: 17px;
            font-weight: bold;
            padding: 8px;
        """)

        control_layout.addWidget(
            self.system_mode
        )

        button_grid = QGridLayout()

        self.start_button = QPushButton(
            'START'
        )

        self.pause_button = QPushButton(
            'PAUSE'
        )

        self.resume_button = QPushButton(
            'RESUME'
        )

        self.stop_button = QPushButton(
            'STOP'
        )

        self.start_button.setStyleSheet(
            self.button_style('#2e7d32')
        )

        self.pause_button.setStyleSheet(
            self.button_style('#ef6c00')
        )

        self.resume_button.setStyleSheet(
            self.button_style('#1565c0')
        )

        self.stop_button.setStyleSheet(
            self.button_style('#b71c1c')
        )

        for button in [
            self.start_button,
            self.pause_button,
            self.resume_button,
            self.stop_button,
        ]:

            button.setMinimumHeight(55)

        button_grid.addWidget(
            self.start_button,
            0, 0
        )

        button_grid.addWidget(
            self.pause_button,
            0, 1
        )

        button_grid.addWidget(
            self.resume_button,
            1, 0
        )

        button_grid.addWidget(
            self.stop_button,
            1, 1
        )

        control_layout.addLayout(
            button_grid
        )

        right.addWidget(
            control_group
        )

        # ========================================================
        # ROBOT STATUS
        # ========================================================

        robot_group = QGroupBox(
            'ROBOT STATUS'
        )

        robot_layout = QGridLayout(
            robot_group
        )

        self.motion_value = QLabel('STOPPED')
        self.linear_value = QLabel('0.000 m/s')
        self.angular_value = QLabel('0.000 rad/s')

        robot_layout.addWidget(
            QLabel('Motion:'),
            0, 0
        )

        robot_layout.addWidget(
            self.motion_value,
            0, 1
        )

        robot_layout.addWidget(
            QLabel('Linear:'),
            1, 0
        )

        robot_layout.addWidget(
            self.linear_value,
            1, 1
        )

        robot_layout.addWidget(
            QLabel('Angular:'),
            2, 0
        )

        robot_layout.addWidget(
            self.angular_value,
            2, 1
        )

        right.addWidget(robot_group)

        # ========================================================
        # LOCALIZATION
        # ========================================================

        localization_group = QGroupBox(
            'LOCALIZATION'
        )

        localization_layout = QGridLayout(
            localization_group
        )

        self.marker_value = QLabel('--')
        self.distance_value = QLabel('--')
        self.localization_value = QLabel('WAITING')

        self.x_value = QLabel('0.000 m')
        self.y_value = QLabel('0.000 m')
        self.yaw_value = QLabel('0.0°')

        self.aruco_rel_x_value = QLabel('--')
        self.aruco_rel_y_value = QLabel('--')
        self.aruco_rel_z_value = QLabel('--')

        localization_layout.addWidget(
            QLabel('ArUco ID:'),
            0, 0
        )

        localization_layout.addWidget(
            self.marker_value,
            0, 1
        )

        localization_layout.addWidget(
            QLabel('Distance:'),
            1, 0
        )

        localization_layout.addWidget(
            self.distance_value,
            1, 1
        )

        localization_layout.addWidget(
            QLabel('Status:'),
            2, 0
        )

        localization_layout.addWidget(
            self.localization_value,
            2, 1
        )

        localization_layout.addWidget(
            QLabel('Odom X:'),
            3, 0
        )

        localization_layout.addWidget(
            self.x_value,
            3, 1
        )

        localization_layout.addWidget(
            QLabel('Odom Y:'),
            4, 0
        )

        localization_layout.addWidget(
            self.y_value,
            4, 1
        )

        localization_layout.addWidget(
            QLabel('Odom Heading:'),
            5, 0
        )

        localization_layout.addWidget(
            self.yaw_value,
            5, 1
        )

        localization_layout.addWidget(
            QLabel('ArUco rel. X:'),
            6, 0
        )
        localization_layout.addWidget(
            self.aruco_rel_x_value,
            6, 1
        )

        localization_layout.addWidget(
            QLabel('ArUco rel. Y:'),
            7, 0
        )
        localization_layout.addWidget(
            self.aruco_rel_y_value,
            7, 1
        )

        localization_layout.addWidget(
            QLabel('ArUco rel. Z:'),
            8, 0
        )
        localization_layout.addWidget(
            self.aruco_rel_z_value,
            8, 1
        )

        right.addWidget(
            localization_group
        )

        # ========================================================
        # INSPECTION
        # ========================================================

        inspection_group = QGroupBox(
            'INSPECTION'
        )

        inspection_layout = QGridLayout(
            inspection_group
        )

        self.inspection_value = QLabel(
            'WAITING'
        )

        self.inspection_pause_value = QLabel(
            'NO'
        )

        self.close_marker_value = QLabel(
            '--'
        )

        inspection_layout.addWidget(
            QLabel('Status:'),
            0, 0
        )

        inspection_layout.addWidget(
            self.inspection_value,
            0, 1
        )

        inspection_layout.addWidget(
            QLabel('Auto Pause:'),
            1, 0
        )

        inspection_layout.addWidget(
            self.inspection_pause_value,
            1, 1
        )

        inspection_layout.addWidget(
            QLabel('Close ID:'),
            2, 0
        )

        inspection_layout.addWidget(
            self.close_marker_value,
            2, 1
        )

        right.addWidget(
            inspection_group
        )

        # ========================================================
        # LOG
        # ========================================================

        log_group = QGroupBox(
            'SYSTEM LOG'
        )

        log_layout = QVBoxLayout(
            log_group
        )

        self.log = QTextEdit()

        self.log.setReadOnly(True)

        self.log.append(
            'GUI started.'
        )

        self.log.append(
            'Monitoring ROS 2 system.'
        )

        log_layout.addWidget(
            self.log
        )

        right.addWidget(
            log_group,
            1
        )

        body.addLayout(
            right,
            1
        )

        # ========================================================
        # BUTTON CONNECTIONS
        # ========================================================

        self.start_button.clicked.connect(
            lambda: self.send_control('START')
        )

        self.pause_button.clicked.connect(
            lambda: self.send_control('PAUSE')
        )

        self.resume_button.clicked.connect(
            lambda: self.send_control('RESUME')
        )

        self.stop_button.clicked.connect(
            lambda: self.send_control('STOP')
        )

        # ========================================================
        # TIMER
        # ========================================================

        self.timer = QTimer()

        self.timer.timeout.connect(
            self.update_gui
        )

        self.timer.start(50)

    # ============================================================
    # GUI HELPERS
    # ============================================================

    def create_image_label(self, text):

        label = QLabel(text)

        label.setAlignment(
            Qt.AlignCenter
        )

        label.setStyleSheet("""
            QLabel {
                background-color: #08090b;
                border: 1px solid #4c566a;
                color: #8f98a8;
            }
        """)

        return label

    def button_style(self, color):

        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 7px;
                padding: 8px;
            }}

            QPushButton:hover {{
                border: 2px solid white;
            }}

            QPushButton:pressed {{
                padding-top: 10px;
            }}
        """

    # ============================================================
    # CONTROL
    # ============================================================

    def send_control(self, command):

        self.ros_node.send_command(
            command
        )

        self.system_mode.setText(
            f'MODE: {command}'
        )

        self.log.append(
            f'Operator command: {command}'
        )

    # ============================================================
    # IMAGE DISPLAY
    # ============================================================

    def display_frame(
        self,
        label,
        frame
    ):

        if frame is None:
            return

        if len(frame.shape) == 2:

            rgb = cv2.cvtColor(
                frame,
                cv2.COLOR_GRAY2RGB
            )

        else:

            rgb = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2RGB
            )

        height, width, channels = rgb.shape

        bytes_per_line = (
            channels * width
        )

        image = QImage(
            rgb.data,
            width,
            height,
            bytes_per_line,
            QImage.Format_RGB888
        )

        pixmap = QPixmap.fromImage(
            image
        )

        pixmap = pixmap.scaled(
            label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )

        label.setPixmap(
            pixmap
        )

    # ============================================================
    # UPDATE
    # ============================================================

    def update_gui(self):

        rclpy.spin_once(
            self.ros_node,
            timeout_sec=0.0
        )

        node = self.ros_node

        # Images
        self.display_frame(
            self.aruco_image,
            node.aruco_frame
        )

        self.display_frame(
            self.line_image,
            node.line_frame
        )

        self.display_frame(
            self.mask_image,
            node.mask_frame
        )

        # ArUco
        if node.aruco_visible and node.aruco_id is not None:

            self.marker_value.setText(
                f'ID {node.aruco_id}'
            )

            self.marker_value.setStyleSheet(
                'color: #4CAF50; font-weight: bold;'
            )

            if node.aruco_id != self.last_marker:

                self.log.append(
                    f'ArUco ID {node.aruco_id} detected'
                )

                self.last_marker = node.aruco_id

        else:

            self.marker_value.setText('--')

        # Close marker
        if node.close_visible and node.close_id is not None:

            self.close_marker_value.setText(
                f'ID {node.close_id}'
            )

            if node.close_id != self.last_close_marker:

                self.log.append(
                    f'Inspection marker ID {node.close_id} reached'
                )

                self.last_close_marker = node.close_id

        else:

            self.close_marker_value.setText('--')

        # Distance
        if node.distance is not None:

            self.distance_value.setText(
                f'{node.distance:.3f} m'
            )

        # Localization
        self.localization_value.setText(
            node.localization_status
        )

        # Position
        self.x_value.setText(
            f'{node.robot_x:.3f} m'
        )

        self.y_value.setText(
            f'{node.robot_y:.3f} m'
        )

        self.yaw_value.setText(
            f'{node.robot_yaw:.1f}°'
        )

        # ArUco marker position relative to the camera
        if node.aruco_visible and node.aruco_rel_x is not None:
            self.aruco_rel_x_value.setText(
                f'{node.aruco_rel_x:.3f} m'
            )
            self.aruco_rel_y_value.setText(
                f'{node.aruco_rel_y:.3f} m'
            )
            self.aruco_rel_z_value.setText(
                f'{node.aruco_rel_z:.3f} m'
            )
        else:
            self.aruco_rel_x_value.setText('--')
            self.aruco_rel_y_value.setText('--')
            self.aruco_rel_z_value.setText('--')

        # Velocity
        self.linear_value.setText(
            f'{node.linear_velocity:.3f} m/s'
        )

        self.angular_value.setText(
            f'{node.angular_velocity:.3f} rad/s'
        )

        moving = (
            abs(node.linear_velocity) > 0.005 or
            abs(node.angular_velocity) > 0.005
        )

        if moving:

            self.motion_value.setText(
                'MOVING'
            )

            self.motion_value.setStyleSheet(
                'color: #4CAF50; font-weight: bold;'
            )

        else:

            self.motion_value.setText(
                'STOPPED'
            )

            self.motion_value.setStyleSheet(
                'color: #FFB300; font-weight: bold;'
            )

        # Inspection
        self.inspection_value.setText(
            node.inspection_status
        )

        if node.inspection_pause:

            self.inspection_pause_value.setText(
                'YES'
            )

            self.inspection_pause_value.setStyleSheet(
                'color: #FFB300; font-weight: bold;'
            )

        else:

            self.inspection_pause_value.setText(
                'NO'
            )

            self.inspection_pause_value.setStyleSheet(
                'color: #4CAF50; font-weight: bold;'
            )

        if node.inspection_status != self.last_inspection_status:

            self.log.append(
                f'Inspection: {node.inspection_status}'
            )

            self.last_inspection_status = (
                node.inspection_status
            )


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    ros_node = HitachiGuiNode()

    app = QApplication(sys.argv)

    window = MainWindow(
        ros_node
    )

    window.show()

    result = app.exec_()

    ros_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(result)


if __name__ == '__main__':
    main()
