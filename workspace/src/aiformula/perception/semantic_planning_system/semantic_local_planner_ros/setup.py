from glob import glob

from setuptools import find_packages, setup

package_name = "semantic_local_planner_ros"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "MAP_ASSIST.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.osm")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="ROS2 Light Avoidance",
    maintainer_email="user@example.com",
    description="Local primitive planner ROS2 node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "local_primitive_planner_node = semantic_local_planner_ros.local_primitive_planner_node:main",
            "planned_lane_points_adapter_node = semantic_local_planner_ros.planned_lane_points_adapter_node:main",
            "lane_points_mux_node = semantic_local_planner_ros.lane_points_mux_node:main",
            "map_assist = semantic_local_planner_ros.map_assist:main",
        ],
    },
)
