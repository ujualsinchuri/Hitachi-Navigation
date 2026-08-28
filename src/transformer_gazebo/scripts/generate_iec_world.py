#!/usr/bin/env python3

"""
Generate an IEC-style transformer inspection world for Gazebo Classic.

Important:
- The uploaded IEC figure does not provide complete transformer dimensions.
- The dimensions below are engineering estimates and are grouped at the top
  so they can be replaced later with measured values.
- The red polygon is the prescribed robot contour.
- The dashed dark polygon is the principal radiating-surface boundary.
- QR markers are intentionally not included in this version.
"""

from pathlib import Path
import math


PACKAGE_DIRECTORY = Path(__file__).resolve().parent.parent
WORLD_PATH = PACKAGE_DIRECTORY / "worlds" / "transformer_iec.world"

# ---------------------------------------------------------------------------
# ESTIMATED TRANSFORMER DIMENSIONS (metres)
# Replace these when real transformer dimensions become available.
# ---------------------------------------------------------------------------
MAIN_TANK_LENGTH = 4.20
MAIN_TANK_WIDTH = 2.20
MAIN_TANK_HEIGHT = 2.40

RADIATOR_BANK_LENGTH = 3.50
RADIATOR_BANK_DEPTH = 0.55
RADIATOR_BANK_HEIGHT = 2.10
RADIATOR_FIN_COUNT = 12

BUSHING_COUNT = 4
BUSHING_RADIUS = 0.10
BUSHING_HEIGHT = 0.75

CONSERVATOR_RADIUS = 0.28
CONSERVATOR_LENGTH = 1.40

# Principal radiating-surface boundary around transformer.
# This outline includes the main tank and the radiator extension.
RADIATING_BOUNDARY = [
    (-2.15, -1.20),
    (-2.15,  1.20),
    ( 2.10,  1.20),
    ( 2.75,  0.95),
    ( 2.75, -0.95),
    ( 2.10, -1.20),
]

# Prescribed contour / robot path.
# Estimated offset is approximately 1.0 m from the radiating surface.
ROBOT_PATH_POINTS = [
    (-3.15, -1.85),
    (-3.20,  1.75),
    (-2.65,  2.35),
    ( 2.35,  2.55),
    ( 3.15,  2.05),
    ( 3.35, -1.55),
    ( 2.65, -2.25),
    (-2.55, -2.45),
]

PATH_LINE_WIDTH = 0.11
PATH_LINE_HEIGHT = 0.008

BOUNDARY_LINE_WIDTH = 0.045
BOUNDARY_LINE_HEIGHT = 0.006
BOUNDARY_DASH_LENGTH = 0.18
BOUNDARY_DASH_GAP = 0.12

# Optional future measurement-point markers.
# Keep False until the real positions/distances are confirmed.
SHOW_MEASUREMENT_POINTS = False
MEASUREMENT_POINT_SPACING = 0.90
MEASUREMENT_MARKER_RADIUS = 0.07

FLOOR_LENGTH = 18.0
FLOOR_WIDTH = 14.0


def box_model(name, x, y, z, length, width, height, color):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{length:.5f} {width:.5f} {height:.5f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{length:.5f} {width:.5f} {height:.5f}</size>
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


def cylinder_model(name, x, y, z, radius, length, color, roll=0.0, pitch=0.0, yaw=0.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.5f} {y:.5f} {z:.5f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose>
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


def visual_line_segment(name, start, end, width, height, color):
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
      <pose>{cx:.5f} {cy:.5f} {height / 2.0:.5f} 0 0 {yaw:.5f}</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <box>
              <size>{length:.5f} {width:.5f} {height:.5f}</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <emissive>{color}</emissive>
          </material>
        </visual>
      </link>
    </model>
    """


def closed_polygon_segments(points):
    for index in range(len(points)):
        yield points[index], points[(index + 1) % len(points)]


def generate_robot_path():
    models = []
    for index, (start, end) in enumerate(closed_polygon_segments(ROBOT_PATH_POINTS)):
        models.append(
            visual_line_segment(
                f"robot_path_{index}",
                start,
                end,
                PATH_LINE_WIDTH,
                PATH_LINE_HEIGHT,
                "1 0 0 1",
            )
        )
    return "\n".join(models)


def generate_dashed_boundary():
    models = []
    model_index = 0

    for start, end in closed_polygon_segments(RADIATING_BOUNDARY):
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)

        if length <= 0.0:
            continue

        ux = dx / length
        uy = dy / length
        distance = 0.0

        while distance < length:
            dash_end_distance = min(distance + BOUNDARY_DASH_LENGTH, length)

            dash_start = (
                x1 + ux * distance,
                y1 + uy * distance,
            )
            dash_end = (
                x1 + ux * dash_end_distance,
                y1 + uy * dash_end_distance,
            )

            models.append(
                visual_line_segment(
                    f"radiating_boundary_{model_index}",
                    dash_start,
                    dash_end,
                    BOUNDARY_LINE_WIDTH,
                    BOUNDARY_LINE_HEIGHT,
                    "0.05 0.05 0.05 1",
                )
            )

            model_index += 1
            distance += BOUNDARY_DASH_LENGTH + BOUNDARY_DASH_GAP

    return "\n".join(models)


def generate_transformer():
    models = []

    tank_color = "0.30 0.32 0.30 1"
    top_color = "0.38 0.40 0.37 1"
    radiator_color = "0.20 0.22 0.20 1"
    bushing_color = "0.25 0.11 0.04 1"
    metal_color = "0.45 0.46 0.43 1"

    # Main tank.
    models.append(
        box_model(
            "main_tank",
            0.0,
            0.0,
            MAIN_TANK_HEIGHT / 2.0,
            MAIN_TANK_LENGTH,
            MAIN_TANK_WIDTH,
            MAIN_TANK_HEIGHT,
            tank_color,
        )
    )

    # Top lid.
    models.append(
        box_model(
            "tank_lid",
            0.0,
            0.0,
            MAIN_TANK_HEIGHT + 0.14,
            MAIN_TANK_LENGTH * 1.03,
            MAIN_TANK_WIDTH * 1.03,
            0.28,
            top_color,
        )
    )

    # Radiator bank on the right side, matching the asymmetric IEC top view.
    radiator_center_x = MAIN_TANK_LENGTH / 2.0 + RADIATOR_BANK_DEPTH / 2.0
    fin_width = RADIATOR_BANK_LENGTH / RADIATOR_FIN_COUNT

    for index in range(RADIATOR_FIN_COUNT):
        y = (
            -RADIATOR_BANK_LENGTH / 2.0
            + fin_width / 2.0
            + index * fin_width
        )

        models.append(
            box_model(
                f"radiator_fin_{index}",
                radiator_center_x,
                y,
                RADIATOR_BANK_HEIGHT / 2.0,
                RADIATOR_BANK_DEPTH,
                fin_width * 0.55,
                RADIATOR_BANK_HEIGHT,
                radiator_color,
            )
        )

    # Two radiator headers.
    for y in (-RADIATOR_BANK_LENGTH / 2.0, RADIATOR_BANK_LENGTH / 2.0):
        models.append(
            cylinder_model(
                f"radiator_header_{'low' if y < 0 else 'high'}",
                radiator_center_x,
                y,
                RADIATOR_BANK_HEIGHT / 2.0,
                0.10,
                RADIATOR_BANK_HEIGHT,
                radiator_color,
            )
        )

    # Bushings on top.
    bushing_spacing = 0.75
    first_x = -((BUSHING_COUNT - 1) * bushing_spacing) / 2.0

    for index in range(BUSHING_COUNT):
        x = first_x + index * bushing_spacing
        y = -0.45

        models.append(
            cylinder_model(
                f"bushing_{index}",
                x,
                y,
                MAIN_TANK_HEIGHT + 0.28 + BUSHING_HEIGHT / 2.0,
                BUSHING_RADIUS,
                BUSHING_HEIGHT,
                bushing_color,
            )
        )

    # Conservator tank, horizontal cylinder near the rear/top.
    models.append(
        cylinder_model(
            "conservator_tank",
            0.55,
            0.45,
            MAIN_TANK_HEIGHT + 0.78,
            CONSERVATOR_RADIUS,
            CONSERVATOR_LENGTH,
            metal_color,
            roll=0.0,
            pitch=math.pi / 2.0,
            yaw=0.0,
        )
    )

    # Simple base rails.
    for y in (-0.82, 0.82):
        models.append(
            box_model(
                f"base_rail_{'left' if y < 0 else 'right'}",
                0.0,
                y,
                0.10,
                MAIN_TANK_LENGTH * 0.90,
                0.12,
                0.20,
                radiator_color,
            )
        )

    return "\n".join(models)


def sample_polygon_by_distance(points, spacing):
    """Sample a closed polygon at approximately uniform path distance."""
    dense = []

    for start, end in closed_polygon_segments(points):
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        count = max(1, math.ceil(length / 0.05))

        for index in range(count):
            t = index / count
            x = x1 + t * dx
            y = y1 + t * dy
            yaw = math.atan2(dy, dx)
            dense.append((x, y, yaw))

    if dense:
        dense.append(dense[0])

    samples = []
    accumulated = 0.0
    previous = dense[0]
    samples.append(previous)

    for point in dense[1:]:
        accumulated += math.hypot(
            point[0] - previous[0],
            point[1] - previous[1],
        )

        if accumulated >= spacing:
            samples.append(point)
            accumulated = 0.0

        previous = point

    return samples


def generate_measurement_markers():
    if not SHOW_MEASUREMENT_POINTS:
        return ""

    models = []
    white = "1 1 1 1"

    for index, (x, y, _) in enumerate(
        sample_polygon_by_distance(
            ROBOT_PATH_POINTS,
            MEASUREMENT_POINT_SPACING,
        )
    ):
        models.append(
            cylinder_model(
                f"measurement_point_{index}",
                x,
                y,
                0.012,
                MEASUREMENT_MARKER_RADIUS,
                0.012,
                white,
            )
        )

    return "\n".join(models)


def generate_world():
    transformer = generate_transformer()
    robot_path = generate_robot_path()
    radiating_boundary = generate_dashed_boundary()
    measurement_markers = generate_measurement_markers()

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="transformer_iec_world">

    <gravity>0 0 -9.81</gravity>

    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.72 0.72 0.72 1</ambient>
      <background>0.88 0.88 0.88 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.95 0.95 0.95 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.35 0.15 -1</direction>
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
            <ambient>0.74 0.74 0.71 1</ambient>
            <diffuse>0.74 0.74 0.71 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    {transformer}

    {radiating_boundary}

    {robot_path}

    {measurement_markers}

    <gui fullscreen="0">
      <camera name="overview_camera">
        <pose>8.5 -10.5 10.5 0 0.67 2.28</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
"""


def main():
    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    WORLD_PATH.write_text(generate_world(), encoding="utf-8")

    print(f"Generated IEC-style world: {WORLD_PATH}")
    print("QR markers are not included.")
    print(f"Measurement markers enabled: {SHOW_MEASUREMENT_POINTS}")
    print(f"Estimated transformer: {MAIN_TANK_LENGTH:.2f} x "
          f"{MAIN_TANK_WIDTH:.2f} x {MAIN_TANK_HEIGHT:.2f} m")


if __name__ == "__main__":
    main()
