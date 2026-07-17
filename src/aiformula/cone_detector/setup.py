from setuptools import setup

package_name = 'cone_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'cone_detector',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@nvidia.com',
    description='Cone detector with YOLOv5 + color',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'cone_detect = cone_detector.cone_detect:main',
        ],
    },
)

