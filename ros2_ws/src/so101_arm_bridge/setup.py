from glob import glob
import os

from setuptools import find_packages, setup


package_name = "so101_arm_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dasom",
    maintainer_email="noreply@github.com",
    description="ROS 2 bridge for the STM32-controlled SO101 dual-arm stream.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bimanual_stream_node = so101_arm_bridge.bimanual_stream_node:main",
        ],
    },
)
