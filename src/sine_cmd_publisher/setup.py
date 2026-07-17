from setuptools import setup

package_name = "sine_cmd_publisher"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="debug",
    maintainer_email="debug@example.com",
    description="Constant-speed sine angular velocity command publisher.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sine_cmd = sine_cmd_publisher.sine_cmd_node:main",
        ],
    },
)
