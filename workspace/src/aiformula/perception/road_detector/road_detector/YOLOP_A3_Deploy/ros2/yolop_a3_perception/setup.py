from setuptools import find_packages, setup

setup(name="yolop_a3_perception", version="0.3.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/yolop_a3_perception"]),
                  ("share/yolop_a3_perception", ["package.xml"])],
      install_requires=["setuptools", "yolop-a3-runtime"],
      entry_points={"console_scripts": ["yolop_a3_node = yolop_a3_perception.node:main"]})
