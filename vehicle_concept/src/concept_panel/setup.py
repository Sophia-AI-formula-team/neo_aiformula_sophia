from setuptools import setup

name = 'concept_panel'
setup(
    name=name, version='6.27.8', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    entry_points={'console_scripts': ['panel = concept_panel.main:main']},
)
