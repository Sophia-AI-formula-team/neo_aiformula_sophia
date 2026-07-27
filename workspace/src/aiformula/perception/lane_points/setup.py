from setuptools import setup

package_name = 'lane_points'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zw01',
    maintainer_email='zw01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                            'lane_twopoint = lane_points.lane_twopoint:main',
                            'lane_zuizhong = lane_points.lane_zuizhong:main',
                            'lane_zuoshuju = lane_points.lane_zuoshuju:main',
                            'lane_new1213 = lane_points.lane_new1213:main',
                            'lane_1213 = lane_points.lane_1213:main',
                            'lane_new0108centercompleted = lane_points.lane_new0108centercompleted:main',
                            'lane_new04 = lane_points.lane_new04:main',
                            'lane_0117 = lane_points.lane_0117:main',
                            'lane_0215 = lane_points.lane_0215:main',
                            'lane_260118 = lane_points.lane_260118:main',
                            'lane_record = lane_points.lane_record:main',
                            'lane_0529oa = lane_points.lane_0529oa:main',
        ],
    },
)
