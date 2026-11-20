from setuptools import setup

package_name = "optimization_oa"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "opti_oa_0_0 = optimization_oa.opti_oa_0_0:main",
        ],
    },
)
