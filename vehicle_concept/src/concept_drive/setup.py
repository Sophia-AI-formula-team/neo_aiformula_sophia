from setuptools import setup

name = 'concept_drive'
setup(
    name=name,
    version='6.26.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    package_data={name: []},
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sophia AI Formula Team',
    maintainer_email='maintainers@example.invalid',
    description='Synchronized sensor-only ROS adapter for V6.26 decisions.',
    license='AGPL-3.0-only',
    entry_points={'console_scripts': ['drive = concept_drive.node:main']},
)
