from glob import glob
from setuptools import find_packages, setup
package_name = "lane_mapping_fixed"
setup(
    name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "LICENSE"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Sophia AI Formula Team", maintainer_email="maintainers@example.invalid",
    description="Fixed-route variant: cease managed LYA after validated stopped handoff.",
    license="MIT", tests_require=["pytest"],
    entry_points={"console_scripts": ["fixed_follower = lane_mapping_fixed.node:main"]},
)
