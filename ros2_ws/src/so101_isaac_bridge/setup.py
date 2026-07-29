from setuptools import find_packages, setup


package_name = "so101_isaac_bridge"

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
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="dasom",
    maintainer_email="noreply@github.com",
    description="SO-101 MoveIt adapter for the Isaac Sim joint-state graph.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_node = so101_isaac_bridge.bridge_node:main",
        ],
    },
)
