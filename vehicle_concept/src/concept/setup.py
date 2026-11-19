from setuptools import setup
from glob import glob

name = 'concept'
setup(
    name=name, version='6.27.6', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/launch', glob('launch/*')),
        ('share/' + name + '/config', glob('config/*.xml')),
    ],
    entry_points={'console_scripts': ['live = concept.live:main', 'replay_sink = concept.replay_sink:main']},
)
