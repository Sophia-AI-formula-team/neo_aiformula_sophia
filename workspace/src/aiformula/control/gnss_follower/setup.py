from setuptools import setup

package_name = 'gnss_follower'

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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'lya_follower_gnss_0303 = gnss_follower.lya_follower_gnss_0303:main',
             'path_gnss_0303 = gnss_follower.path_gnss_0303:main',
             'lya_follower_gnss_0315 = gnss_follower.lya_follower_gnss_0315:main',
             'lya_gnss_plus_opencv0316 = gnss_follower.lya_gnss_plus_opencv0316:main',
             'lya_gnss_plus_path0316 = gnss_follower.lya_gnss_plus_path0316:main',
             'lya_gnss_base0316 = gnss_follower.lya_gnss_base0316:main',
             'path_gnss_0315 = gnss_follower.path_gnss_0315:main',
             
        ],
    },
)
