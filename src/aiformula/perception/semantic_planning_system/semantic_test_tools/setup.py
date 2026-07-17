from setuptools import find_packages, setup

package_name = "semantic_test_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    scripts=["scripts/run_core_demo.py"],
    zip_safe=True,
    maintainer="ROS2 Light Avoidance",
    maintainer_email="user@example.com",
    description="Fake test publishers for semantic local planning.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fake_mask_publisher_node = semantic_test_tools.fake_mask_publisher_node:main",
            "fake_traffic_light_node = semantic_test_tools.fake_traffic_light_node:main",
            "fake_lane_result_node = semantic_test_tools.fake_lane_result_node:main",
        ],
    },
)
