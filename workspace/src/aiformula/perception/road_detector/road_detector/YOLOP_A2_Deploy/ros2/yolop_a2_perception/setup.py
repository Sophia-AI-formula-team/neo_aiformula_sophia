from setuptools import find_packages, setup


PACKAGE_NAME = "yolop_a2_perception"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sophia AI Formula Team",
    maintainer_email="sophia-ai-formula@example.invalid",
    description="ROS2 wrapper for the inference-only YOLOP_A2 runtime.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolop_a2_node = yolop_a2_perception.node:main",
        ],
    },
)
