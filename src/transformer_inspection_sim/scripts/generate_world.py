#!/usr/bin/env python3

from pathlib import Path
import csv
import math


PACKAGE_DIRECTORY = Path(__file__).resolve().parent.parent
WORLD_PATH = PACKAGE_DIRECTORY / "worlds" / "Transformer.world"
CSV_PATH = PACKAGE_DIRECTORY / "config" / "aruco_locations.csv"

TRANSFORMER_X = 0.0
TRANSFORMER_Y = 1.98

TRANSFORMER_LENGTH = 3.50
TRANSFORMER_WIDTH = 2.00
TRANSFORMER_HEIGHT = 2.40

PATH_LEFT = TRANSFORMER_X - 2.75
PATH_RIGHT = TRANSFORMER_X + 3.15
PATH_BOTTOM = TRANSFORMER_Y - 2.00
PATH_TOP = TRANSFORMER_Y + 2.00
PATH_CHAMFER = 0.65

PATH_LINE_WIDTH = 0.10
PATH_LINE_HEIGHT = 0.008

ARUCO_Z = 0.018

FLOOR_LENGTH = 20.0
FLOOR_WIDTH = 16.0


PATH_POINTS = [
    (PATH_LEFT + PATH_CHAMFER, PATH_BOTTOM),
    (PATH_RIGHT - PATH_CHAMFER, PATH_BOTTOM),
    (PATH_RIGHT, PATH_BOTTOM + PATH_CHAMFER),
    (PATH_RIGHT, PATH_TOP - PATH_CHAMFER),
    (PATH_RIGHT - PATH_CHAMFER, PATH_TOP),
    (PATH_LEFT + PATH_CHAMFER, PATH_TOP),
    (PATH_LEFT, PATH_TOP - PATH_CHAMFER),
    (PATH_LEFT, PATH_BOTTOM + PATH_CHAMFER),
]


def box_model(name, x, y, z, sx, sy, sz, color):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """


def cylinder_model(name, x, y, z, radius, length, color, pitch=0.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.5f} {y:.5f} {z:.5f} 0 {pitch:.6f} 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius:.5f}</radius>
              <length>{length:.5f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius:.5f}</radius>
              <length>{length:.5f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """


def line_segment(name, start, end):
    x1, y1 = start
    x2, y2 = end

    dx = x2 - x1
    dy = y2 - y1

    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)

    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx:.5f} {cy:.5f} {PATH_LINE_HEIGHT / 2.0:.5f} 0 0 {yaw:.6f}</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <box>
              <size>{length:.5f} {PATH_LINE_WIDTH:.5f} {PATH_LINE_HEIGHT:.5f}</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <emissive>1 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
    """


def interpolate(start, end, ratio):
    return (
        start[0] + ratio * (end[0] - start[0]),
        start[1] + ratio * (end[1] - start[1]),
    )


def inspection_points():
    raw_points = []

    bottom_left = PATH_POINTS[0]
    bottom_right = PATH_POINTS[1]

    right_lower = PATH_POINTS[2]
    right_upper = PATH_POINTS[3]

    top_right = PATH_POINTS[4]
    top_left = PATH_POINTS[5]

    left_upper = PATH_POINTS[6]
    left_lower = PATH_POINTS[7]

    for ratio in (0.08, 0.29, 0.50, 0.71, 0.92):
        x, y = interpolate(bottom_left, bottom_right, ratio)
        raw_points.append((x, y, 0.0))

    for ratio in (0.34, 0.68):
        x, y = interpolate(right_lower, right_upper, ratio)
        raw_points.append((x, y, math.pi / 2.0))

    for ratio in (0.08, 0.29, 0.50, 0.71, 0.92):
        x, y = interpolate(top_right, top_left, ratio)
        raw_points.append((x, y, math.pi))

    for ratio in (0.34, 0.68):
        x, y = interpolate(left_upper, left_lower, ratio)
        raw_points.append((x, y, -math.pi / 2.0))

    points = []

    for marker_id, (x, y, yaw) in enumerate(raw_points):
        points.append(
            (
                marker_id,
                x,
                y,
                yaw,
                x,
                y,
                yaw + math.pi / 2.0,
            )
        )

    return points


def marker_include(marker_id, x, y, yaw):
    model_name = f"aruco_floor_{marker_id:02d}"

    return f"""
    <include>
      <uri>model://{model_name}</uri>
      <name>{model_name}</name>
      <pose>{x:.5f} {y:.5f} {ARUCO_Z:.5f} 0 0 {yaw:.6f}</pose>
    </include>
    """


def generate_transformer():
    models = []

    models.append(
        box_model(
            "transformer_tank",
            TRANSFORMER_X,
            TRANSFORMER_Y,
            TRANSFORMER_HEIGHT / 2.0,
            TRANSFORMER_LENGTH,
            TRANSFORMER_WIDTH,
            TRANSFORMER_HEIGHT,
            "0.34 0.36 0.34 1",
        )
    )

    models.append(
        box_model(
            "transformer_lid",
            TRANSFORMER_X,
            TRANSFORMER_Y,
            TRANSFORMER_HEIGHT + 0.12,
            TRANSFORMER_LENGTH * 1.07,
            TRANSFORMER_WIDTH * 1.10,
            0.24,
            "0.45 0.46 0.43 1",
        )
    )

    radiator_x = (
        TRANSFORMER_X
        + TRANSFORMER_LENGTH / 2.0
        + 0.20
    )

    for index in range(13):
        y = (
            TRANSFORMER_Y
            - 0.92
            + index * (1.84 / 12.0)
        )

        models.append(
            box_model(
                f"radiator_fin_{index:02d}",
                radiator_x,
                y,
                1.05,
                0.40,
                0.07,
                2.10,
                "0.18 0.19 0.18 1",
            )
        )

    for index, x_offset in enumerate(
        (-1.05, -0.35, 0.35, 1.05)
    ):
        models.append(
            cylinder_model(
                f"bushing_{index}",
                TRANSFORMER_X + x_offset,
                TRANSFORMER_Y - 0.40,
                2.95,
                0.10,
                0.75,
                "0.31 0.13 0.05 1",
            )
        )

    models.append(
        cylinder_model(
            "conservator",
            TRANSFORMER_X + 0.55,
            TRANSFORMER_Y + 0.48,
            3.22,
            0.25,
            1.35,
            "0.48 0.49 0.46 1",
            pitch=math.pi / 2.0,
        )
    )

    return "\n".join(models)


def generate_path():
    segments = []

    for index in range(len(PATH_POINTS)):
        segments.append(
            line_segment(
                f"inspection_path_{index}",
                PATH_POINTS[index],
                PATH_POINTS[(index + 1) % len(PATH_POINTS)],
            )
        )

    return "\n".join(segments)


def generate_markers():
    return "\n".join(
        marker_include(
            marker_id,
            marker_x,
            marker_y,
            marker_yaw,
        )
        for (
            marker_id,
            _,
            _,
            _,
            marker_x,
            marker_y,
            marker_yaw,
        ) in inspection_points()
    )


def write_csv():
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)

    with CSV_PATH.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow(
            [
                "marker_id",
                "x",
                "y",
                "yaw",
                "marker_x",
                "marker_y",
                "marker_yaw",
            ]
        )

        for row in inspection_points():
            writer.writerow(
                [
                    row[0],
                    f"{row[1]:.5f}",
                    f"{row[2]:.5f}",
                    f"{row[3]:.5f}",
                    f"{row[4]:.5f}",
                    f"{row[5]:.5f}",
                    f"{row[6]:.5f}",
                ]
            )


def generate_world():
    write_csv()

    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="transformer_world">

    <gravity>0 0 -9.81</gravity>

    <physics name="ode_physics" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <shadows>true</shadows>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.85 0.85 0.85 1</background>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.95 0.95 1</diffuse>
      <direction>-0.4 0.2 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{FLOOR_LENGTH} {FLOOR_WIDTH}</size>
            </plane>
          </geometry>
        </collision>

        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{FLOOR_LENGTH} {FLOOR_WIDTH}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.72 0.72 0.70 1</ambient>
            <diffuse>0.72 0.72 0.70 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    {generate_transformer()}

    {generate_path()}

    {generate_markers()}

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>8 -8 8 0 0.65 2.30</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
"""


def main():
    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)

    WORLD_PATH.write_text(
        generate_world(),
        encoding="utf-8",
    )

    print(f"Generated world: {WORLD_PATH}")
    print(f"Generated ArUco CSV: {CSV_PATH}")
    print(f"ArUco markers: {len(inspection_points())}")


if __name__ == "__main__":
    main()
