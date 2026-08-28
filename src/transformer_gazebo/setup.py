from glob import glob
import os

from setuptools import find_packages, setup


package_name = "transformer_gazebo"


def collect_tree(source_directory):
    data_files = []

    if not os.path.isdir(source_directory):
        return data_files

    for current_directory, _, filenames in os.walk(source_directory):
        if not filenames:
            continue

        install_directory = os.path.join(
            "share",
            package_name,
            current_directory,
        )

        source_files = [
            os.path.join(current_directory, filename)
            for filename in filenames
        ]

        data_files.append((install_directory, source_files))

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
    version="0.4.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kwd",
    maintainer_email="kwd@example.com",
    description=(
        "Infographic-matched transformer environment "
        "with 14 QR measurement landmarks."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
