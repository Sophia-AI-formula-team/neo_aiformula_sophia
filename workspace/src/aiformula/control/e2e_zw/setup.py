from setuptools import setup

package_name = 'e2e_zw'

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
        'e2erun_node = e2e_zw.e2erun_node:main',
        'e2erun_node_02 = e2e_zw.e2erun_node_02:main',
        'e2erun_node_03 = e2e_zw.e2erun_node_03:main',
        ],
    },
)
