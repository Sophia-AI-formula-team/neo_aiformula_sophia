from setuptools import find_packages, setup

package_name = "semantic_costmap_ros"

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
    description="Semantic costmap ROS2 node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "local_costmap_node = semantic_costmap_ros.local_costmap_node:main",
            "color_semantic_mask_node = semantic_costmap_ros.color_semantic_mask_node:main",
        ],
    },
)
