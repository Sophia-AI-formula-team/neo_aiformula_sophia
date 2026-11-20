from setuptools import setup

setup(
    name="oa_1b",
    version="0.1.0",
    packages=["oa_1b"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/oa_1b"]),
        ("share/oa_1b", ["package.xml"]),
        ("share/oa_1b/tools", ["tools/prepare_control.py", "tools/red_flag.patch"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Sophia AI Formula Team",
    maintainer_email="skromosk@gmail.com",
    description="Vehicle OA 1B experiment for the 1121 field preparation",
    license="AGPL-3.0-or-later",
)
