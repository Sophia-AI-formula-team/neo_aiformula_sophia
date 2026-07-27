from setuptools import find_packages, setup

package_name = "semantic_lane_recovery_ros"

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
    description="Lane recovery gate ROS2 node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lane_recovery_gate_node = semantic_lane_recovery_ros.lane_recovery_gate_node:main",
        ],
    },
)
