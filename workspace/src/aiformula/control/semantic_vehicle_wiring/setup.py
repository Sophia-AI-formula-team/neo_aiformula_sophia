from setuptools import find_packages, setup

setup(name="semantic_vehicle_wiring", version="0.2.0", packages=find_packages(exclude=["tests"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/semantic_vehicle_wiring"]),
                  ("share/semantic_vehicle_wiring", ["package.xml", "README.md"])],
      install_requires=["setuptools"], python_requires=">=3.8", license="AGPL-3.0-or-later",
      entry_points={"console_scripts": ["a3_signal_capture = semantic_vehicle_wiring.capture:main",
          "motor_gate_session = semantic_vehicle_wiring.session:main",
          "semantic = semantic_vehicle_wiring.session:semantic_main"]})
