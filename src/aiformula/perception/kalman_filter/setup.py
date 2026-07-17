from setuptools import setup

package_name = 'kalman_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Yu Narukami',
    maintainer_email='1037657394@qq.com',
    description='A ROS2 package implementing Kalman Filter for smoothing lane points and angles',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_filter_node = kalman_filter.kalman_filter_node:main',
            'kalman_filter_node1 = kalman_filter.kalman_filter_node1:main',
            'kalman_filter_node2 = kalman_filter.kalman_filter_node2:main',
            'kalman_filter_node3 = kalman_filter.kalman_filter_node3:main',
            'kalman_filter_node4 = kalman_filter.kalman_filter_node4:main',
            'kalman0117 = kalman_filter.kalman0117:main',
            'kalman0225 = kalman_filter.kalman0225:main',
            'withoutkalman = kalman_filter.withoutkalman:main',
            'kalman_record = kalman_filter.kalman_record:main',
            'withoutkalman_0312 = kalman_filter.withoutkalman_0312:main',
            'zvz = kalman_filter.zvz:main',
            'omegatest = kalman_filter.omegatest:main',
        ],
    },
)
