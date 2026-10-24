from glob import glob
from setuptools import find_packages, setup

package_name = "lane_mapping_lya_reference"
setup(
    name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "VALIDATION.md", "LICENSE"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Sophia AI Formula Team", maintainer_email="maintainers@example.invalid",
    description="Causal lane mapping and fixed-route following with live LYA safety reference.",
    license="MIT", tests_require=["pytest"],
    entry_points={"console_scripts": [
        "lap_recorder = lane_mapping_lya_reference.recorder_node:main",
        "reference_follower = lane_mapping_lya_reference.follower_node:main_reference",
        "teacher_supervisor = lane_mapping_lya_reference.supervisor:main",
    ]},
)
