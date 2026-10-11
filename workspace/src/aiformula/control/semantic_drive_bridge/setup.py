from setuptools import find_packages, setup

package_name = "semantic_drive_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="Sophia AI Formula Team",
    maintainer_email="skromosk@gmail.com",
    description="Stamped signal ingress and shadow-only command watchdog using real rclpy.",
    license="AGPL-3.0-or-later",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "signal_capture_bridge = semantic_drive_bridge.nodes:signal_capture_main",
        "command_watchdog = semantic_drive_bridge.nodes:command_watchdog_main",
    ]},
)
