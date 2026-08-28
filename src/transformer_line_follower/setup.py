from setuptools import find_packages, setup


package_name = "transformer_line_follower"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kwd",
    maintainer_email="kwd@example.com",
    description=(
        "Red-line following controller for a TurtleBot3 "
        "transformer inspection simulation."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "line_follower = "
                "transformer_line_follower.line_follower:main"
            ),
        ],
    },
)
