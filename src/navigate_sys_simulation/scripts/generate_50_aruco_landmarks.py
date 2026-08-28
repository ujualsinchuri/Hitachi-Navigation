#!/usr/bin/env python3

"""
Generate 50 ArUco localization landmarks (IDs 0..49) around the existing
transformer red-line route.

Important design choices:
- Uses DICT_4X4_50, so valid IDs are exactly 0..49.
- Keeps markers away from sharp corners.
- Centers each marker ON the red line.
- Uses smaller 0.20 m markers to reduce line occlusion.
- Reuses aruco_floor_00 as the Gazebo model template.
- Updates BOTH Transformer.world and aruco_locations.csv.
"""

from pathlib import Path
import csv
import math
import re
import shutil

import cv2


PKG = Path(__file__).resolve().parent.parent

WORLD = PKG / "worlds" / "Transformer.world"
CSV_PATH = PKG / "config" / "aruco_locations.csv"
MODELS = PKG / "models"

TEMPLATE_MODEL = MODELS / "aruco_floor_00"

NUMBER_OF_MARKERS = 50
MARKER_SIZE_M = 0.20

# Keep localization markers away from the actual turn point.
CORNER_CLEARANCE_M = 0.55

# Current red-line centerline reconstructed from the working Line.stl.
PATH_POINTS = [
    (-4.468873, -9.271854),
    (-3.578786, -9.919190),
    (-1.610921, -9.919190),
    ( 4.844358, -9.402768),
    ( 5.075578,  1.634137),
    ( 3.479587,  4.959118),
    ( 1.119455,  6.208600),
    (-1.335851,  6.208600),
    (-6.334590,  2.809458),
    (-6.334590,  0.205989),
]


def distance(a, b):
    return math.hypot(
        b[0] - a[0],
        b[1] - a[1],
    )


def interpolate(a, b, t):
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
    )


def segment_yaw(a, b):
    return math.atan2(
        b[1] - a[1],
        b[0] - a[0],
    )


def allocate_counts(weights, total):
    """
    Allocate integer marker counts proportional to usable segment length.
    """
    raw = [
        total * w / sum(weights)
        for w in weights
    ]

    base = [
        int(math.floor(x))
        for x in raw
    ]

    remaining = total - sum(base)

    order = sorted(
        range(len(raw)),
        key=lambda i: raw[i] - base[i],
        reverse=True,
    )

    for i in order[:remaining]:
        base[i] += 1

    return base


def find_texture_png(model_dir):
    pngs = list(
        model_dir.rglob("*.png")
    )

    if not pngs:
        raise RuntimeError(
            f"No PNG texture found inside template {model_dir}"
        )

    return pngs[0]


def make_marker_image(marker_id, output_png):
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    image = cv2.aruco.generateImageMarker(
        dictionary,
        marker_id,
        800,
    )

    output_png.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    cv2.imwrite(
        str(output_png),
        image,
    )


def generate_safe_poses():
    """
    Allocate markers on segment interiors only.
    No marker is placed directly at a vertex/corner.
    """

    closed = (
        PATH_POINTS
        + [PATH_POINTS[0]]
    )

    segment_info = []

    for i in range(
        len(PATH_POINTS)
    ):
        a = closed[i]
        b = closed[i + 1]

        length = distance(a, b)

        usable = max(
            0.0,
            length
            - 2.0 * CORNER_CLEARANCE_M,
        )

        segment_info.append(
            (
                a,
                b,
                length,
                usable,
            )
        )

    usable_lengths = [
        item[3]
        for item in segment_info
    ]

    counts = allocate_counts(
        usable_lengths,
        NUMBER_OF_MARKERS,
    )

    poses = []

    marker_id = 0

    for (
        segment_index,
        (
            a,
            b,
            length,
            usable,
        ),
    ) in enumerate(
        segment_info
    ):

        count = counts[
            segment_index
        ]

        if count <= 0:
            continue

        start_distance = (
            CORNER_CLEARANCE_M
        )

        end_distance = (
            length
            - CORNER_CLEARANCE_M
        )

        if count == 1:
            distances = [
                (
                    start_distance
                    + end_distance
                ) / 2.0
            ]
        else:
            spacing = (
                end_distance
                - start_distance
            ) / (
                count - 1
            )

            distances = [
                start_distance
                + spacing * j
                for j in range(
                    count
                )
            ]

        yaw = segment_yaw(
            a,
            b,
        )

        for d in distances:

            t = d / length

            x, y = interpolate(
                a,
                b,
                t,
            )

            marker_yaw = (
                yaw
                + math.pi / 2.0
            )

            poses.append(
                (
                    marker_id,
                    x,
                    y,
                    yaw,
                    x,
                    y,
                    marker_yaw,
                )
            )

            marker_id += 1

    if marker_id != NUMBER_OF_MARKERS:
        raise RuntimeError(
            f"Generated {marker_id} markers, expected {NUMBER_OF_MARKERS}"
        )

    return poses


def rebuild_marker_models(poses):
    """Regenerate all 50 marker models from one consistent definition.

    This avoids stale model.sdf sizes and stale PNG resolutions from older
    copies of the package.
    """
    import subprocess
    import sys

    generator = PKG / "scripts" / "generate_aruco_models.py"
    subprocess.run([sys.executable, str(generator)], check=True)

    # Safety check: every generated SDF must use the configured 0.20 m size.
    expected = f"<size>{MARKER_SIZE_M} {MARKER_SIZE_M}</size>"
    for marker_id, *_ in poses:
        sdf = MODELS / f"aruco_floor_{marker_id:02d}" / "model.sdf"
        if expected not in sdf.read_text(encoding="utf-8"):
            raise RuntimeError(f"Unexpected marker size in {sdf}")


def write_csv(poses):
    CSV_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with CSV_PATH.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as f:

        writer = csv.writer(
            f
        )

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

        for (
            marker_id,
            x,
            y,
            yaw,
            marker_x,
            marker_y,
            marker_yaw,
        ) in poses:

            writer.writerow(
                [
                    marker_id,
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{yaw:.6f}",
                    f"{marker_x:.6f}",
                    f"{marker_y:.6f}",
                    f"{marker_yaw:.6f}",
                ]
            )


def update_world(poses):
    world_text = WORLD.read_text(
        encoding="utf-8"
    )

    # Remove all existing ArUco include blocks.
    pattern = re.compile(
        r"""
        \s*<include>\s*
        <uri>model://aruco_floor_\d+</uri>\s*
        <name>aruco_floor_\d+</name>\s*
        <pose>[^<]+</pose>\s*
        </include>
        """,
        re.MULTILINE
        | re.VERBOSE,
    )

    world_text = pattern.sub(
        "",
        world_text,
    )

    include_blocks = []

    for (
        marker_id,
        _x,
        _y,
        _yaw,
        marker_x,
        marker_y,
        marker_yaw,
    ) in poses:

        include_blocks.append(
            f"""
    <include>
      <uri>model://aruco_floor_{marker_id:02d}</uri>
      <name>aruco_floor_{marker_id:02d}</name>
      <pose>{marker_x:.6f} {marker_y:.6f} 0.035 0 0 {marker_yaw:.6f}</pose>
    </include>
"""
        )

    marker_text = "".join(
        include_blocks
    )

    if "</world>" not in world_text:
        raise RuntimeError(
            "Transformer.world has no </world> closing tag"
        )

    world_text = world_text.replace(
        "</world>",
        marker_text
        + "\n  </world>",
        1,
    )

    WORLD.write_text(
        world_text,
        encoding="utf-8",
    )


def main():
    if not WORLD.is_file():
        raise FileNotFoundError(
            WORLD
        )

    # Backups first.
    shutil.copy2(
        WORLD,
        WORLD.with_suffix(
            ".world.before_50_markers"
        ),
    )

    if CSV_PATH.is_file():
        shutil.copy2(
            CSV_PATH,
            CSV_PATH.with_suffix(
                ".csv.before_50_markers"
            ),
        )

    poses = generate_safe_poses()

    rebuild_marker_models(
        poses
    )

    write_csv(
        poses
    )

    update_world(
        poses
    )

    print()
    print(
        "Generated 50 ArUco localization markers successfully."
    )
    print(
        "IDs: 0 through 49"
    )
    print(
        f"Marker size: {MARKER_SIZE_M:.2f} m"
    )
    print(
        f"Corner clearance: {CORNER_CLEARANCE_M:.2f} m"
    )
    print()
    print(
        f"Updated: {WORLD}"
    )
    print(
        f"Updated: {CSV_PATH}"
    )
    print()
    print(
        "IMPORTANT: use the selected-ID inspection manager so the robot "
        "does not stop at all 50 localization markers."
    )


if __name__ == "__main__":
    main()
