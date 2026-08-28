#!/usr/bin/env python3

from pathlib import Path
import csv
import math


PACKAGE_DIRECTORY = Path(__file__).resolve().parent.parent
WORLD_PATH = PACKAGE_DIRECTORY / "worlds" / "transformer_line.world"
CSV_PATH = PACKAGE_DIRECTORY / "config" / "qr_locations.csv"

# Transformer dimensions in metres.
TRANSFORMER_LENGTH = 7.0
TRANSFORMER_WIDTH = 4.0
TRANSFORMER_HEIGHT = 4.2

# Red path settings.
PATH_CLEARANCE = 3.0
CORNER_RADIUS = 1.2
LINE_WIDTH = 0.12
LINE_HEIGHT = 0.006
LINE_SEGMENT_LENGTH = 0.16

# QR settings.
QR_SPACING = 1.0
QR_SIDE_OFFSET = 0.35
QR_Z = 0.012

FLOOR_LENGTH = 24.0
FLOOR_WIDTH = 20.0


def box_model(
    name,
    x,
    y,
    z,
    length,
    width,
    height,
    color,
):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{length} {width} {height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{length} {width} {height}</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """


def cylinder_model(
    name,
    x,
    y,
    z,
    radius,
    length,
    color,
):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{length}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{length}</length>
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


def line_segment(name, x, y, yaw, length):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.5f} {y:.5f} {LINE_HEIGHT / 2.0:.5f} 0 0 {yaw:.5f}</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <box>
              <size>{length:.5f} {LINE_WIDTH:.5f} {LINE_HEIGHT:.5f}</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <emissive>0.35 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
    """


def qr_include(index, x, y, yaw):
    model_name = f"qr_marker_{index:03d}"

    return f"""
    <include>
      <uri>model://{model_name}</uri>
      <name>{model_name}</name>
      <pose>{x:.5f} {y:.5f} {QR_Z:.5f} 0 0 {yaw:.5f}</pose>
    </include>
    """


def append_straight_line(
    models,
    name_prefix,
    start_x,
    start_y,
    end_x,
    end_y,
):
    dx = end_x - start_x
    dy = end_y - start_y
    distance = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)

    count = max(1, math.ceil(distance / LINE_SEGMENT_LENGTH))
    segment_length = distance / count

    for index in range(count):
        ratio = (index + 0.5) / count
        x = start_x + ratio * dx
        y = start_y + ratio * dy

        models.append(
            line_segment(
                f"{name_prefix}_{index}",
                x,
                y,
                yaw,
                segment_length + 0.01,
            )
        )


def append_arc(
    models,
    name_prefix,
    centre_x,
    centre_y,
    radius,
    start_angle,
    end_angle,
):
    arc_length = abs(end_angle - start_angle) * radius
    count = max(4, math.ceil(arc_length / LINE_SEGMENT_LENGTH))
    angle_step = (end_angle - start_angle) / count
    segment_length = abs(radius * angle_step) + 0.02

    for index in range(count):
        angle = start_angle + (index + 0.5) * angle_step
        x = centre_x + radius * math.cos(angle)
        y = centre_y + radius * math.sin(angle)
        yaw = angle + math.pi / 2.0

        models.append(
            line_segment(
                f"{name_prefix}_{index}",
                x,
                y,
                yaw,
                segment_length,
            )
        )


def generate_red_path():
    models = []

    half_length = TRANSFORMER_LENGTH / 2.0
    half_width = TRANSFORMER_WIDTH / 2.0

    outer_x = half_length + PATH_CLEARANCE
    outer_y = half_width + PATH_CLEARANCE
    radius = min(CORNER_RADIUS, outer_x - 0.2, outer_y - 0.2)

    append_straight_line(
        models,
        "path_bottom",
        -outer_x + radius,
        -outer_y,
        outer_x - radius,
        -outer_y,
    )
    append_arc(
        models,
        "path_corner_br",
        outer_x - radius,
        -outer_y + radius,
        radius,
        -math.pi / 2.0,
        0.0,
    )
    append_straight_line(
        models,
        "path_right",
        outer_x,
        -outer_y + radius,
        outer_x,
        outer_y - radius,
    )
    append_arc(
        models,
        "path_corner_tr",
        outer_x - radius,
        outer_y - radius,
        radius,
        0.0,
        math.pi / 2.0,
    )
    append_straight_line(
        models,
        "path_top",
        outer_x - radius,
        outer_y,
        -outer_x + radius,
        outer_y,
    )
    append_arc(
        models,
        "path_corner_tl",
        -outer_x + radius,
        outer_y - radius,
        radius,
        math.pi / 2.0,
        math.pi,
    )
    append_straight_line(
        models,
        "path_left",
        -outer_x,
        outer_y - radius,
        -outer_x,
        -outer_y + radius,
    )
    append_arc(
        models,
        "path_corner_bl",
        -outer_x + radius,
        -outer_y + radius,
        radius,
        math.pi,
        3.0 * math.pi / 2.0,
    )

    return "\n".join(models)


def rounded_rectangle_samples():
    """Return path samples at approximately QR_SPACING metres."""
    half_length = TRANSFORMER_LENGTH / 2.0
    half_width = TRANSFORMER_WIDTH / 2.0

    outer_x = half_length + PATH_CLEARANCE
    outer_y = half_width + PATH_CLEARANCE
    radius = min(CORNER_RADIUS, outer_x - 0.2, outer_y - 0.2)

    segments = [
        (
            "straight",
            (-outer_x + radius, -outer_y),
            (outer_x - radius, -outer_y),
        ),
        (
            "arc",
            (outer_x - radius, -outer_y + radius),
            radius,
            -math.pi / 2.0,
            0.0,
        ),
        (
            "straight",
            (outer_x, -outer_y + radius),
            (outer_x, outer_y - radius),
        ),
        (
            "arc",
            (outer_x - radius, outer_y - radius),
            radius,
            0.0,
            math.pi / 2.0,
        ),
        (
            "straight",
            (outer_x - radius, outer_y),
            (-outer_x + radius, outer_y),
        ),
        (
            "arc",
            (-outer_x + radius, outer_y - radius),
            radius,
            math.pi / 2.0,
            math.pi,
        ),
        (
            "straight",
            (-outer_x, outer_y - radius),
            (-outer_x, -outer_y + radius),
        ),
        (
            "arc",
            (-outer_x + radius, -outer_y + radius),
            radius,
            math.pi,
            3.0 * math.pi / 2.0,
        ),
    ]

    dense_points = []

    for segment in segments:
        if segment[0] == "straight":
            _, start, end = segment
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            count = max(2, math.ceil(length / 0.05))

            for index in range(count):
                ratio = index / count
                x = start[0] + ratio * dx
                y = start[1] + ratio * dy
                yaw = math.atan2(dy, dx)
                dense_points.append((x, y, yaw))
        else:
            _, centre, arc_radius, start_angle, end_angle = segment
            arc_length = abs(end_angle - start_angle) * arc_radius
            count = max(4, math.ceil(arc_length / 0.05))

            for index in range(count):
                ratio = index / count
                angle = start_angle + ratio * (end_angle - start_angle)
                x = centre[0] + arc_radius * math.cos(angle)
                y = centre[1] + arc_radius * math.sin(angle)
                yaw = angle + math.pi / 2.0
                dense_points.append((x, y, yaw))

    if dense_points:
        dense_points.append(dense_points[0])

    sampled = []
    accumulated = 0.0
    previous = dense_points[0]
    sampled.append(previous)

    for point in dense_points[1:]:
        distance = math.hypot(
            point[0] - previous[0],
            point[1] - previous[1],
        )
        accumulated += distance

        if accumulated >= QR_SPACING:
            sampled.append(point)
            accumulated = 0.0

        previous = point

    return sampled


def generate_qr_markers():
    includes = []
    locations = []

    for index, (path_x, path_y, path_yaw) in enumerate(
        rounded_rectangle_samples()
    ):
        # Place each QR marker exactly on the centre of the red path.
        qr_x = path_x
        qr_y = path_y

        includes.append(
            qr_include(index, qr_x, qr_y, path_yaw)
        )

        locations.append(
            (
                f"QR_{index:03d}",
                qr_x,
                qr_y,
                path_yaw,
            )
        )

    return "\n".join(includes), locations

def generate_transformer():
    models = []

    grey = "0.28 0.29 0.27 1"
    dark_grey = "0.18 0.19 0.18 1"
    metal = "0.42 0.43 0.40 1"

    body_height = TRANSFORMER_HEIGHT * 0.72

    models.append(
        box_model(
            "transformer_main_body",
            0.0,
            0.0,
            body_height / 2.0,
            TRANSFORMER_LENGTH,
            TRANSFORMER_WIDTH,
            body_height,
            grey,
        )
    )

    models.append(
        box_model(
            "transformer_top",
            0.0,
            0.0,
            body_height + 0.25,
            TRANSFORMER_LENGTH * 0.88,
            TRANSFORMER_WIDTH * 0.88,
            0.50,
            metal,
        )
    )

    radiator_length = TRANSFORMER_LENGTH * 0.72
    radiator_width = 0.45
    radiator_height = body_height * 0.82

    models.append(
        box_model(
            "radiator_left",
            0.0,
            TRANSFORMER_WIDTH / 2.0 + radiator_width / 2.0,
            radiator_height / 2.0,
            radiator_length,
            radiator_width,
            radiator_height,
            dark_grey,
        )
    )

    models.append(
        box_model(
            "radiator_right",
            0.0,
            -(TRANSFORMER_WIDTH / 2.0 + radiator_width / 2.0),
            radiator_height / 2.0,
            radiator_length,
            radiator_width,
            radiator_height,
            dark_grey,
        )
    )

    models.append(
        cylinder_model(
            "conservator_tank",
            0.5,
            0.0,
            TRANSFORMER_HEIGHT + 0.25,
            0.43,
            2.30,
            grey,
        )
    )

    for index, (x, y) in enumerate(
        [
            (-2.1, -0.9),
            (-0.7, -0.9),
            (0.7, -0.9),
            (2.1, -0.9),
        ]
    ):
        models.append(
            cylinder_model(
                f"transformer_bushing_{index}",
                x,
                y,
                TRANSFORMER_HEIGHT + 0.45,
                0.13,
                0.90,
                "0.20 0.14 0.08 1",
            )
        )

    return "\n".join(models)


def write_csv(locations):
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)

    with CSV_PATH.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["marker_id", "x", "y", "yaw"])

        for marker_id, x, y, yaw in locations:
            writer.writerow(
                [
                    marker_id,
                    f"{x:.5f}",
                    f"{y:.5f}",
                    f"{yaw:.5f}",
                ]
            )


def generate_world():
    transformer_models = generate_transformer()
    path_models = generate_red_path()
    qr_models, locations = generate_qr_markers()

    write_csv(locations)

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="transformer_world">

    <gravity>0 0 -9.81</gravity>

    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.70 0.70 0.70 1</ambient>
      <background>0.85 0.85 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.95 0.95 0.95 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="factory_floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry>
            <box>
              <size>{FLOOR_LENGTH} {FLOOR_WIDTH} 0.10</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry>
            <box>
              <size>{FLOOR_LENGTH} {FLOOR_WIDTH} 0.10</size>
            </box>
          </geometry>
          <material>
            <ambient>0.72 0.72 0.69 1</ambient>
            <diffuse>0.72 0.72 0.69 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    {transformer_models}

    {path_models}

    {qr_models}

    <gui fullscreen="0">
      <camera name="overview_camera">
        <pose>11 -13 14 0 0.65 2.35</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
"""


def main():
    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    WORLD_PATH.write_text(generate_world(), encoding="utf-8")

    print(f"Generated world: {WORLD_PATH}")
    print(f"Generated QR position table: {CSV_PATH}")


if __name__ == "__main__":
    main()
