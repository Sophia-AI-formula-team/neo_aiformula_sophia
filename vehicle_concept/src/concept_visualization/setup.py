from setuptools import setup
from glob import glob

name = 'concept_visualization'
setup(
    name=name, version='6.27.4', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/rviz', glob('rviz/*')),
    ],
    entry_points={'console_scripts': ['visualization = concept_visualization.node:main']},
)
