from setuptools import find_packages, setup

PACKAGE = "aiformula_frontend_demo"

setup(name=PACKAGE, version="0.1.0", packages=find_packages(),
      data_files=[
          ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
          ("share/" + PACKAGE, ["package.xml"]),
          ("share/" + PACKAGE + "/launch", ["launch/replay.launch.py"]),
      ],
      install_requires=["setuptools"], zip_safe=False,
      description="Independent camera semantics, geometric registration and opti_oa demo nodes.",
      maintainer="AI Formula", maintainer_email="dev@example.invalid",
      license="Apache-2.0",
      entry_points={"console_scripts": [
          "semantic = aiformula_frontend_demo.semantic_node:main",
          "geometric_hash = aiformula_frontend_demo.geometry_node:main",
          "opti_oa = aiformula_frontend_demo.opti_oa_node:main",
      ]})
