from setuptools import setup

setup(
    name="oa_1b",
    version="0.1.1",
    packages=["oa_1b"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/oa_1b"]),
        ("share/oa_1b", ["package.xml"]),
        ("share/oa_1b/launch", ["launch/mapassist_b.launch.py"]),
    ],
)
