from setuptools import find_packages, setup

package_name = "semantic_local_planner_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS2 Light Avoidance",
    maintainer_email="user@example.com",
    description="Local primitive planner ROS2 node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "local_primitive_planner_node = semantic_local_planner_ros.local_primitive_planner_node:main",
        ],
    },
)
