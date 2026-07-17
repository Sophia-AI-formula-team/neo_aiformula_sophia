from setuptools import setup, find_packages
import os

package_name = 'traffic_cones_detection_ros'


def collect_package_data(pkg_dir_rel):
    """
    收集 package_dir_rel 下所有文件，返回相对路径列表（用于 package_data）。
    pkg_dir_rel 例如：traffic_cones_detection_ros/vendor/traffic_cones_detection
    """
    out = []
    for root, _, files in os.walk(pkg_dir_rel):
        for f in files:
            full = os.path.join(root, f)
            rel = os.path.relpath(full, package_name)  # 相对到 traffic_cones_detection_ros/ 目录
            out.append(rel)
    return out


# 你 vendor repo 的位置必须是：
# traffic_cones_detection_ros/vendor/traffic_cones_detection/...
vendor_rel = os.path.join(package_name, 'vendor', 'traffic_cones_detection')

# 把 vendor 下所有文件（包括 best.pt）加入安装
pkg_data = collect_package_data(vendor_rel)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={package_name: pkg_data},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@nvidia.com',
    description='Vendor package for traffic_cones_detection (YOLOv5 + color)',
    license='Apache-2.0',
)

