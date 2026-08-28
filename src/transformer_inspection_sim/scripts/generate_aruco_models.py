#!/usr/bin/env python3

from pathlib import Path
import shutil

import cv2
import numpy as np


PACKAGE_DIRECTORY = Path(__file__).resolve().parent.parent
MODELS_DIRECTORY = PACKAGE_DIRECTORY / "models"

NUMBER_OF_MARKERS = 14
MARKER_SIZE_METRES = 0.50
MARKER_IMAGE_PIXELS = 900
QUIET_BORDER_PIXELS = 120
ARUCO_DICTIONARY_NAME = "DICT_4X4_50"


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>kwd</name></author>
  <description>Floor ArUco marker ID {marker_id}</description>
</model>
"""


MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="aruco_visual">
        <pose>0 0 0.005 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{marker_size} {marker_size}</size>
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://{model_name}/materials/scripts</uri>
            <uri>model://{model_name}/materials/textures</uri>
            <name>{material_name}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


MATERIAL = """material {material_name}
{{
  technique
  {{
    pass
    {{
      lighting off
      depth_write on
      cull_hardware none
      cull_software none
      texture_unit
      {{
        texture {texture_filename}
        filtering none
      }}
    }}
  }}
}}
"""


def get_dictionary():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable. Install OpenCV with ArUco support."
        )

    dictionary_id = getattr(
        cv2.aruco,
        ARUCO_DICTIONARY_NAME,
    )

    return cv2.aruco.getPredefinedDictionary(
        dictionary_id
    )


def draw_marker(dictionary, marker_id):
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(
            dictionary,
            marker_id,
            MARKER_IMAGE_PIXELS,
        )
    else:
        marker = np.zeros(
            (MARKER_IMAGE_PIXELS, MARKER_IMAGE_PIXELS),
            dtype=np.uint8,
        )

        cv2.aruco.drawMarker(
            dictionary,
            marker_id,
            MARKER_IMAGE_PIXELS,
            marker,
            1,
        )

    return cv2.copyMakeBorder(
        marker,
        QUIET_BORDER_PIXELS,
        QUIET_BORDER_PIXELS,
        QUIET_BORDER_PIXELS,
        QUIET_BORDER_PIXELS,
        cv2.BORDER_CONSTANT,
        value=255,
    )


def generate_marker(dictionary, marker_id):
    model_name = f"aruco_floor_{marker_id:02d}"
    material_name = f"ArUco/Floor{marker_id:02d}"
    texture_filename = f"aruco_{marker_id:02d}.png"

    model_directory = MODELS_DIRECTORY / model_name
    script_directory = model_directory / "materials" / "scripts"
    texture_directory = model_directory / "materials" / "textures"

    script_directory.mkdir(parents=True, exist_ok=True)
    texture_directory.mkdir(parents=True, exist_ok=True)

    marker_image = draw_marker(dictionary, marker_id)
    texture_path = texture_directory / texture_filename

    if not cv2.imwrite(str(texture_path), marker_image):
        raise RuntimeError(f"Failed to write {texture_path}")

    (model_directory / "model.config").write_text(
        MODEL_CONFIG.format(
            model_name=model_name,
            marker_id=marker_id,
        ),
        encoding="utf-8",
    )

    (model_directory / "model.sdf").write_text(
        MODEL_SDF.format(
            model_name=model_name,
            marker_size=MARKER_SIZE_METRES,
            material_name=material_name,
        ),
        encoding="utf-8",
    )

    (script_directory / "aruco.material").write_text(
        MATERIAL.format(
            material_name=material_name,
            texture_filename=texture_filename,
        ),
        encoding="utf-8",
    )


def main():
    MODELS_DIRECTORY.mkdir(parents=True, exist_ok=True)

    for old_directory in MODELS_DIRECTORY.glob("aruco_floor_*"):
        if old_directory.is_dir():
            shutil.rmtree(old_directory)

    dictionary = get_dictionary()

    for marker_id in range(NUMBER_OF_MARKERS):
        generate_marker(dictionary, marker_id)

    print(
        f"Generated {NUMBER_OF_MARKERS} markers using "
        f"{ARUCO_DICTIONARY_NAME}"
    )


if __name__ == "__main__":
    main()
