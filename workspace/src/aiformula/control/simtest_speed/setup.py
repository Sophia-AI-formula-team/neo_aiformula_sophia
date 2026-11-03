from setuptools import setup

package_name = 'simtest_speed'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simtest',
    maintainer_email='simtest@example.com',
    description='Straight-line speed test',
    license='UNLICENSED',
    entry_points={
        'console_scripts': [
            'speed = simtest_speed.speed:main',
        ],
    },
)
