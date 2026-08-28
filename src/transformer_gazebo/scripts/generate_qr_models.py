#!/usr/bin/env python3

from pathlib import Path
import shutil

import qrcode


PACKAGE_DIRECTORY = Path(__file__).resolve().parent.parent
MODELS_DIRECTORY = PACKAGE_DIRECTORY / "models"

NUMBER_OF_MARKERS = 14
MARKER_SIZE = 0.34


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>kwd</name>
  </author>
  <description>{marker_id}</description>
</model>
"""


MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="qr_visual">
        <pose>0 0 0.003 0 0 0</pose>
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
        texture qr.png
        filtering none
      }}
    }}
  }}
}}
"""


def generate_marker(index: int) -> None:
    marker_number = index + 1
    marker_id = f"QR_{marker_number:02d}"
    model_name = f"qr_marker_{marker_number:02d}"
    material_name = f"QR/Marker{marker_number:02d}"

    model_directory = MODELS_DIRECTORY / model_name
    script_directory = model_directory / "materials" / "scripts"
    texture_directory = model_directory / "materials" / "textures"

    script_directory.mkdir(parents=True, exist_ok=True)
    texture_directory.mkdir(parents=True, exist_ok=True)

    qr_code = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=28,
        border=4,
    )
    qr_code.add_data(marker_id)
    qr_code.make(fit=True)

    image = qr_code.make_image(
        fill_color="black",
        back_color="white",
    ).convert("RGB")

    image.save(texture_directory / "qr.png")

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
            marker_size=MARKER_SIZE,
            material_name=material_name,
        ),
        encoding="utf-8",
    )

    (script_directory / "qr.material").write_text(
        MATERIAL.format(material_name=material_name),
        encoding="utf-8",
    )


def main() -> None:
    MODELS_DIRECTORY.mkdir(parents=True, exist_ok=True)

    for old_directory in MODELS_DIRECTORY.glob("qr_marker_*"):
        if old_directory.is_dir():
            shutil.rmtree(old_directory)

    for index in range(NUMBER_OF_MARKERS):
        generate_marker(index)

    print(
        f"Generated {NUMBER_OF_MARKERS} QR marker models in "
        f"{MODELS_DIRECTORY}"
    )


if __name__ == "__main__":
    main()
