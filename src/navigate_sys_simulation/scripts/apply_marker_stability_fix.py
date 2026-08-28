#!/usr/bin/env python3
from pathlib import Path
import shutil

PKG = Path.home() / "ros2_ws" / "src" / "navigate_sys" / "navigate_sys"
LINE = PKG / "line_follower.py"
ARUCO = PKG / "aruco_localizer.py"

for p in (LINE, ARUCO):
    if not p.is_file():
        raise FileNotFoundError(p)
    shutil.copy2(p, p.with_suffix(p.suffix + ".bak"))

a = ARUCO.read_text()

a = a.replace(
    "from std_msgs.msg import Int32",
    "from std_msgs.msg import Bool, Int32",
)

needle = """        self.debug_publisher = (
            self.create_publisher(
                Image,
                "/aruco_debug_image",
                10,
            )
        )
"""
replacement = needle + """
        self.visible_publisher = (
            self.create_publisher(
                Bool,
                "/aruco_visible",
                10,
            )
        )
        self.close_publisher = (
            self.create_publisher(
                Bool,
                "/aruco_close_visible",
                10,
            )
        )
"""
if needle not in a:
    raise RuntimeError("Aruco publisher block not found")
a = a.replace(needle, replacement, 1)

needle = """        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                debug,
                corners,
                ids,
            )

            for marker_corners, raw_id in zip(
                corners,
                ids.flatten(),
            ):
"""
replacement = """        any_visible = False
        any_close = False

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
                any_visible = True
"""
if needle not in a:
    raise RuntimeError("Aruco loop block not found")
a = a.replace(needle, replacement, 1)

needle = """                if (
                    close
                    and marker_id
                    in self.marker_poses
                ):
                    self.publish_marker(
                        marker_id,
                        message,
                    )

        debug_message = (
"""
replacement = """                if close:
                    any_close = True

                if (
                    close
                    and marker_id
                    in self.marker_poses
                ):
                    self.publish_marker(
                        marker_id,
                        message,
                    )

        visible_msg = Bool()
        visible_msg.data = bool(any_visible)
        self.visible_publisher.publish(visible_msg)

        close_msg = Bool()
        close_msg.data = bool(any_close)
        self.close_publisher.publish(close_msg)

        debug_message = (
"""
if needle not in a:
    raise RuntimeError("Aruco publish block not found")
a = a.replace(needle, replacement, 1)
ARUCO.write_text(a)

l = LINE.read_text()

needle = """        self.create_subscription(
            Bool,
            "/inspection_pause",
            self.pause_callback,
            10,
        )

        self.paused = False
"""
replacement = """        self.create_subscription(
            Bool,
            "/inspection_pause",
            self.pause_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/aruco_visible",
            self.aruco_visible_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/aruco_close_visible",
            self.aruco_close_callback,
            10,
        )

        self.paused = False
        self.aruco_visible = False
        self.aruco_close = False
        self.marker_pass_hold = False
        self.marker_clear_frames = 0
"""
if needle not in l:
    raise RuntimeError("Line subscriber block not found")
l = l.replace(needle, replacement, 1)

needle = "    def make_red_mask(\n"
replacement = """    def aruco_visible_callback(self, msg: Bool):
        self.aruco_visible = bool(msg.data)

    def aruco_close_callback(self, msg: Bool):
        self.aruco_close = bool(msg.data)

    def marker_near_target(self, mask):
        height, _ = mask.shape[:2]
        y1 = int(height * 0.52)
        y2 = int(height * 0.70)
        points = cv2.findNonZero(mask[y1:y2, :])

        if points is None or len(points) < 20:
            return None

        xs = points[:, 0, 0]
        ys = points[:, 0, 1]

        return (
            float(np.median(xs)),
            float(np.median(ys) + y1),
        )

""" + needle
if needle not in l:
    raise RuntimeError("Line helper insertion point not found")
l = l.replace(needle, replacement, 1)

l = l.replace(
    "            self.stop()\n            return\n\n        # PAUSED -> RUNNING",
    "            self.marker_pass_hold = True\n            self.stop()\n            return\n\n        # PAUSED -> RUNNING",
    1,
)

l = l.replace(
    "            self.resume_reacquire_frames = 18",
    "            self.marker_pass_hold = True\n            self.marker_clear_frames = 0\n            self.resume_reacquire_frames = 0",
    1,
)

needle = """        mask = self.make_red_mask(
            frame
        )

        path = self.trace_connected_path(
            mask
        )
"""
replacement = """        mask = self.make_red_mask(
            frame
        )

        if self.marker_pass_hold and not self.aruco_visible:
            self.marker_clear_frames += 1
            if self.marker_clear_frames >= 12:
                self.marker_pass_hold = False
                self.marker_clear_frames = 0
                self.filtered_error = 0.0
                self.previous_error = 0.0
                self.filtered_angular = 0.0
        elif self.aruco_visible:
            self.marker_clear_frames = 0

        marker_mode = (
            self.aruco_visible
            or self.aruco_close
            or self.marker_pass_hold
        )

        if marker_mode:
            command = Twist()
            target = self.marker_near_target(mask)
            command.linear.x = 0.013

            if target is None:
                command.angular.z = 0.0
            else:
                current_error = (
                    target[0] - width / 2.0
                ) / (width / 2.0)
                command.angular.z = float(
                    np.clip(
                        self.steering_sign * 0.22 * current_error,
                        -0.08,
                        0.08,
                    )
                )

            self.filtered_error = 0.0
            self.previous_error = 0.0
            self.filtered_angular = 0.0
            self.last_good_angular = 0.0
            self.last_control_time = time.monotonic()

            state = (
                "MARKER CLOSE PASS"
                if self.aruco_close
                else (
                    "MARKER APPROACH"
                    if self.aruco_visible
                    else "POST-MARKER PASS"
                )
            )

            self.cmd_pub.publish(command)
            self.publish_debug(
                frame,
                mask,
                None,
                state,
                command,
                msg,
            )
            return

        path = self.trace_connected_path(
            mask
        )
"""
if needle not in l:
    raise RuntimeError("Line marker-mode insertion point not found")
l = l.replace(needle, replacement, 1)

LINE.write_text(l)

compile(ARUCO.read_text(), str(ARUCO), "exec")
compile(LINE.read_text(), str(LINE), "exec")

print("Marker stability patch applied.")
print(f"Backup: {LINE}.bak")
print(f"Backup: {ARUCO}.bak")
