from setuptools import setup

name = 'concept_candidates'
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
    description='V6.26 forward trajectories, contingencies and reverse recovery.',
    license='AGPL-3.0-only',
)
