from setuptools import setup

package_name = 'trajectory_follower'

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
             'p_follower = trajectory_follower.p_follower:main',
             'p_follower_connected = trajectory_follower.p_follower_connected:main',
             'pid_follower = trajectory_follower.pid_follower:main',
             'pid_follower_connected = trajectory_follower.pid_follower_connected:main',
             'pid_follower_connected_global = trajectory_follower.pid_follower_connected_global:main',
             'lya_follower = trajectory_follower.lya_follower:main',
             'lya_follower_connected = trajectory_follower.lya_follower_connected:main',
             'lya_follower_connected_global = trajectory_follower.lya_follower_connected_global:main',       'lya_follower_connected_omegat_global=trajectory_follower.lya_follower_connected_omegat_global:main',
             'lya_follower_0122=trajectory_follower.lya_follower_0122:main',
             'lya_record=trajectory_follower.lya_record:main',
             'lya_superrecord=trajectory_follower.lya_superrecord:main',
             'lya_superrecord2=trajectory_follower.lya_superrecord2:main',
             'enhanced_lya_record=trajectory_follower.enhanced_lya_record:main',
             'lya_0221=trajectory_follower.lya_0221:main',
             'lya_precircle=trajectory_follower.lya_precircle:main',
             'lya_preset=trajectory_follower.lya_preset:main',
             'pid_preset=trajectory_follower.pid_preset:main',
             'pp_preset=trajectory_follower.pp_preset:main',
             'lya_avoid_preset=trajectory_follower.lya_avoid_preset:main',
             'lya_oa=trajectory_follower.lya_oa:main',
             'lya_follower_fixedpath=trajectory_follower.lya_follower_fixedpath:main',
             'lya_follower_fixedpath_record=trajectory_follower.lya_follower_fixedpath_record:main',
             'lya_baseline_follower_fixedpath_record=trajectory_follower.lya_baseline_follower_fixedpath_record:main',
        ],
    },
)
