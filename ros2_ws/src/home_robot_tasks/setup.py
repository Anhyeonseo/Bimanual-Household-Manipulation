from setuptools import find_packages, setup

setup(
    name="home_robot_tasks",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/home_robot_tasks"]),
        ("share/home_robot_tasks", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="dasom",
    maintainer_email="noreply@github.com",
    description="Hardware-independent household task requests and planning.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["plan_fetch = home_robot_tasks.cli:main"]},
)
