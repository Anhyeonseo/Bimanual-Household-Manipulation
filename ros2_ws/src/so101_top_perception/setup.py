import os
from glob import glob

from setuptools import find_packages, setup


package_name = "so101_top_perception"

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
        ("share/" + package_name, ["README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="dasom",
    maintainer_email="noreply@github.com",
    description="Fail-closed board-relative Top-camera object pose detector.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "top_object_pose_node = so101_top_perception.node:main",
            "top_shadow_target_node = so101_top_perception.shadow_node:main",
        ],
    },
)
