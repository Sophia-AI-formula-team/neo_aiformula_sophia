from setuptools import setup

package_name = 'obsticle_avoidence'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='zw01',
    maintainer_email='zw01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'oa_neo = obsticle_avoidence.oa_neo:main',
        'oa_neo_nurbs = obsticle_avoidence.oa_neo_nurbs:main',
        'oa_neo_ubs = obsticle_avoidence.oa_neo_ubs:main',
        'odom_path_recorder = obsticle_avoidence.odom_path_recorder:main',
        'data_record = obsticle_avoidence.data_record:main',
        ],
    },
)
