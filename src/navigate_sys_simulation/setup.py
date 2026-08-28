from glob import glob
import os

from setuptools import find_packages, setup


package_name = "navigate_sys"


def collect_tree(folder):
    data_files = []

    if not os.path.isdir(folder):
        return data_files

    for current_directory, _, filenames in os.walk(folder):
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
        os.path.join(
            "share",
            package_name,
            "launch",
        ),
        glob("launch/*.launch.py"),
    ),
    (
        os.path.join(
            "share",
            package_name,
            "worlds",
        ),
        glob("worlds/*.world"),
    ),
]

data_files += collect_tree("models")
data_files += collect_tree("config")


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(
        exclude=["test"]
    ),
    data_files=data_files,
    install_requires=[
        "setuptools"
    ],
    zip_safe=True,
    maintainer="kwd",
    maintainer_email="kwd@example.com",
    description=(
        "Autonomous TurtleBot3 transformer "
        "navigation and inspection system."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "line_follower = "
                "navigate_sys.line_follower:main"
            ),
            (
                "aruco_localizer = "
                "navigate_sys.aruco_localizer:main"
            ),
            (
                "inspection_manager = "
                "navigate_sys.inspection_manager:main"
            ),
            (
                "localization_monitor = "
                "navigate_sys.localization_monitor:main"
            ),
        ],
    },
)
