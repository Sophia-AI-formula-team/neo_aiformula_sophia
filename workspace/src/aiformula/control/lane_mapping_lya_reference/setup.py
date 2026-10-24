from glob import glob
import hashlib
from pathlib import Path
import sys

from setuptools import Extension, find_packages, setup

package_name = "lane_mapping_lya_reference"
native_source = "native/lane_graph_native.c"
native_sha256 = hashlib.sha256(Path(native_source).read_bytes()).hexdigest()
setup(
    name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "VALIDATION.md", "LICENSE"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    ext_modules=[Extension(
        "_lane_mapping_lya_native", sources=[native_source],
        define_macros=[("SOURCE_SHA256", '"' + native_sha256 + '"')],
        extra_compile_args=["/O2", "/W4"] if sys.platform == "win32" else ["-O3", "-Wall", "-Wextra"],
    )],
    install_requires=["setuptools"], zip_safe=False,
    maintainer="Sophia AI Formula Team", maintainer_email="maintainers@example.invalid",
    description="Causal lane mapping and fixed-route following with live LYA safety reference.",
    license="MIT", tests_require=["pytest"],
    entry_points={"console_scripts": [
        "lap_recorder = lane_mapping_lya_reference.recorder_node:main",
        "reference_follower = lane_mapping_lya_reference.follower_node:main_reference",
        "teacher_supervisor = lane_mapping_lya_reference.supervisor:main",
    ]},
)
