from glob import glob
import os

from setuptools import find_packages, setup


package_name = "transformer_inspection_sim"


def collect_tree(source_directory):
    data_files = []

    if not os.path.isdir(source_directory):
        return data_files

    for current_directory, _, filenames in os.walk(source_directory):
        if not filenames:
            continue

        data_files.append(
            (
                os.path.join(
                    "share",
                    package_name,
                    current_directory,
                ),
                [
                    os.path.join(
                        current_directory,
                        filename,
                    )
                    for filename in filenames
                ],
            )
        )

    return data_files


data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    (
        "share/" + package_name,
        ["package.xml"],
    ),
    (
        os.path.join("share", package_name, "launch"),
        glob("launch/*.launch.py"),
    ),
    (
        os.path.join("share", package_name, "worlds"),
        glob("worlds/*.world"),
    ),
]

data_files += collect_tree("models")
data_files += collect_tree("config")


setup(
    name=package_name,
    version="5.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kwd",
    maintainer_email="kwd@example.com",
    description="Complete TurtleBot3 transformer inspection simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "line_follower = transformer_inspection_sim.line_follower:main",
            "aruco_localizer = transformer_inspection_sim.aruco_localizer:main",
            "inspection_manager = transformer_inspection_sim.inspection_manager:main",
        ],
    },
)
