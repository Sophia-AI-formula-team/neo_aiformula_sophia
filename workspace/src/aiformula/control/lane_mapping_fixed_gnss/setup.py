from glob import glob
from setuptools import find_packages, setup

name = "lane_mapping_fixed_gnss"
setup(
    name=name, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml", "README.md"]),
        ("share/" + name + "/launch", glob("launch/*.launch.py")),
        ("share/" + name + "/config", glob("config/*.yaml")),
        ("share/" + name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Sophia AI Formula Team", maintainer_email="maintainers@example.invalid",
    description="Endpoint GNSS anchors, causal wheel/raw gyro/mask mapping and fixed-route control.",
    license="MIT", tests_require=["pytest"],
    entry_points={"console_scripts": [
        "endpoint_follower = lane_mapping_fixed_gnss.node:main",
    ]},
)
