from setuptools import setup

package_name = 'simtest_turn'

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
    description='Constant-speed steering test',
    license='UNLICENSED',
    entry_points={
        'console_scripts': [
            'turn = simtest_turn.turn:main',
        ],
    },
)
